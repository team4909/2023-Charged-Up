package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    private final PhotonCamera m_camera;
    private final AprilTagFieldLayout m_aprilTagFieldLayout = loadLayout();
    private final PhotonPoseEstimator m_photonPoseEstimator;

    private static Vision m_instance = null;

    private SimulatedCamera m_simCameraInstance = null;

    private Vision() {
        if (Constants.SIM) {
            m_simCameraInstance = SimulatedCamera.createInstance();
            m_simCameraInstance.addVisionTargets(m_aprilTagFieldLayout);
        }

        m_camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        m_photonPoseEstimator = new PhotonPoseEstimator(
                m_aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                m_camera,
                getCameraTransform());

    }

    private AprilTagFieldLayout loadLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public void periodic() {
        if (m_simCameraInstance != null)
            m_simCameraInstance.updatePose.accept(new Pose2d()); // TODO add pose
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousEstimatedPose) {
        m_photonPoseEstimator.setReferencePose(previousEstimatedPose);
        return m_photonPoseEstimator.update();
    }

    // #region Util
    public static Transform3d getCameraTransform() {
        Translation3d centerLocation = new Translation3d(
                VisionConstants.ROBOT_SIDE / 2, VisionConstants.ROBOT_SIDE / 2, VisionConstants.ROBOT_HEIGHT / 2);
        Translation3d camLocationWithRespectToBackRightCorner = new Translation3d(
                VisionConstants.CAMERA_X, VisionConstants.CAMERA_Y, VisionConstants.CAMERA_Z);
        return new Transform3d(
                camLocationWithRespectToBackRightCorner.minus(centerLocation),
                new Rotation3d(0d, VisionConstants.CAMERA_PITCH_RADIANS, 0d));

    }
    // #endregion

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

}
