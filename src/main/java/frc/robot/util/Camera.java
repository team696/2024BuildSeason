package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Camera {
    private static Camera m_Camera;

    private AprilTagFieldLayout m_atLayout; 
    private PhotonCamera m_PCamera;
    private PhotonPoseEstimator m_Estimator;

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

        m_PCamera = new PhotonCamera("Cam 1");
        m_Estimator = new PhotonPoseEstimator(m_atLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_PCamera, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        m_Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updatePose(SwerveDrivePoseEstimator estimator) {
        Optional<EstimatedRobotPose> estimation = m_Estimator.update();
        if (estimation.isPresent()) {
            estimator.addVisionMeasurement(estimation.get().estimatedPose.toPose2d(), estimation.get().timestampSeconds);
        }
    }
}
