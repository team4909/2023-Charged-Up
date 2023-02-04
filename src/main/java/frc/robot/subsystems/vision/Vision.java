package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.camera.CameraBase;
import frc.robot.subsystems.vision.camera.LimelightCamera;
import frc.robot.subsystems.vision.camera.SimulatedCamera;

public class Vision extends SubsystemBase {

    private final CameraBase m_camera;
    private PhotonPoseEstimator m_photonPoseEstimator;

    public Vision() {
        m_camera = Constants.SIM ? new SimulatedCamera() : new LimelightCamera();

        m_photonPoseEstimator = new PhotonPoseEstimator(
                m_camera.getAprilTagFieldLayout(),
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
                m_camera,
                null);

    }

}
