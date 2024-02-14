package frc.robot.util;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Swerve;

public class Camera {
    private static Camera m_Camera;

    private AprilTagFieldLayout m_atLayout; 
    private PhotonCamera m_PCamera;
    private PhotonPoseEstimator m_Estimator;

    private VisionSystemSim m_simVision;
    private PhotonCameraSim m_SCamera;

    Comparator<PhotonTrackedTarget> AmbiguityCompare = new Comparator<PhotonTrackedTarget>() {
        @Override
        public int compare(PhotonTrackedTarget o1, PhotonTrackedTarget o2) {
            return (int)((o1.getPoseAmbiguity() - o2.getPoseAmbiguity()) * 100);
        }
    };

    public static synchronized Camera get() {
        if (m_Camera == null) {
            m_Camera = new Camera();
        }
        return m_Camera;
    }
    
    private Camera() {
        try {
            m_atLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            Log.fatalException("Camera", "Failed to load april tag layout.", e);
        }

        m_PCamera = new PhotonCamera(Constants.Cameras.name);
        m_Estimator = new PhotonPoseEstimator(m_atLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_PCamera, Constants.Cameras.position);
        m_Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        PortForwarder.add(5800, "photonvision.local", 5800);
        Log.info("Photonvision", "Initialized: Dashboard open at http://photonvision.local:5800");

        if (Constants.DEBUG) {
            for(AprilTag tag : m_atLayout.getTags()) { // Shows tags in layout where they should be
                Constants.Field.sim.getObject(tag.ID + "Desired").setPose(tag.pose.toPose2d());
            }
        }
    }

    public void simInit() {
        m_simVision = new VisionSystemSim("photonvision");
        m_SCamera = new PhotonCameraSim(m_PCamera);
        m_simVision.addCamera(m_SCamera, Constants.Cameras.position);
        m_simVision.addAprilTags(m_atLayout);
    }

    public void simPeriodic() {
        m_simVision.update(Swerve.get().getPose());
    }

    public void updatePose(SwerveDrivePoseEstimator estimator) {
        if (RobotBase.isSimulation()) return;

        Optional<EstimatedRobotPose> estimation = m_Estimator.update();
        if (estimation.isPresent()) {
            List<PhotonTrackedTarget> targets = estimation.get().targetsUsed;
            targets.sort(AmbiguityCompare);
            if (Constants.DEBUG) { 
                for (PhotonTrackedTarget t : targets) { // Shows where tags are believed to be based on robot pose.
                    Transform3d targetTransform = t.getBestCameraToTarget(); 
                    Pose2d target = new Pose2d(targetTransform.getTranslation().rotateBy(Constants.Cameras.position.getRotation()).toTranslation2d(), targetTransform.getRotation().toRotation2d());
                    Transform2d balls = new Transform2d(target.getTranslation(), target.getRotation());
                    Pose2d tagToCam = Swerve.get().getPose().transformBy(balls);
                    Constants.Field.sim.getObject(String.valueOf(t.getFiducialId())).setPose(tagToCam.plus(new Transform2d(Constants.Cameras.position.getTranslation().toTranslation2d().rotateBy(new Rotation2d(Math.PI)), Constants.Cameras.position.getRotation().toRotation2d())));
                }
            }
            PhotonTrackedTarget bestTarget = targets.get(0);
            if (bestTarget.getPoseAmbiguity() > 0.13) return; // Too Ambiguous, Ignore
            //if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > 4) return; // Tag Too far, Ignore
            double deviationRatio; 
            if (bestTarget.getPoseAmbiguity() < 1/100.0) {
                deviationRatio = 1/100.0; // Tag estimation very good -> Use it
            } else {
                deviationRatio = Math.pow(bestTarget.getBestCameraToTarget().getTranslation().getNorm(),2) / 6; // Trust Less With Distance
            }
            Matrix<N3, N1> deviation = VecBuilder.fill(deviationRatio, deviationRatio, 5 * deviationRatio);
            estimator.setVisionMeasurementStdDevs(deviation);
            estimator.addVisionMeasurement(estimation.get().estimatedPose.toPose2d(), estimation.get().timestampSeconds);
        }
    }
}
