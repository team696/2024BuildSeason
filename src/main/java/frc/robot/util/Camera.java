package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
        Log.info("Photonvision", "Initialized: Dashboard open at photonvision.local:5800.");
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
            PhotonTrackedTarget bestTarget = estimation.get().targetsUsed.get(0);//m_PCamera.getLatestResult().getBestTarget(); //
            if (bestTarget.getPoseAmbiguity() > 0.15) return;
            if (bestTarget.getBestCameraToTarget().getTranslation().getNorm() > 4) return;
            double deviationRatio; 
            if (bestTarget.getPoseAmbiguity() < 0.01) {
                deviationRatio = 1/100.0;
            } else {
                deviationRatio = bestTarget.getBestCameraToTarget().getTranslation().getNorm() * 3;
            }
            Matrix<N3, N1> deviation = VecBuilder.fill(0.3 * deviationRatio, 0.3 * deviationRatio, 0.6 * deviationRatio);
            estimator.setVisionMeasurementStdDevs(deviation);
            estimator.addVisionMeasurement(estimation.get().estimatedPose.toPose2d(), estimation.get().timestampSeconds);
        }
    }
}
