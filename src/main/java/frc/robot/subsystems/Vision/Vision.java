package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final PhotonCamera m_photonCamera;
    private final PhotonPoseEstimator m_photonPoseEstimator;
    private AprilTagFieldLayout m_aprilTagFieldLayout;

    public Vision() {

        m_aprilTagFieldLayout = null;
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // poseEstimation = new PhotonPoseEstimator(
        // aprilTagFieldLayout, // Field layout
        // PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        // camera,
        // ROBOT_TO_CAM_DIST);

        m_photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        m_photonPoseEstimator = new PhotonPoseEstimator(
                m_aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                m_photonCamera,
                null);

    }

    // public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    // poseEstimation.setReferencePose(new Pose3d(5, 5, 0, new Rotation3d(0, 0,
    // 0)));
    // return poseEstimation.update();
    // }
}
