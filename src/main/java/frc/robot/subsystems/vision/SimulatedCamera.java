package frc.robot.subsystems.vision;

import java.util.function.Consumer;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

public final class SimulatedCamera {

    private final double m_minTargetArea = 10d;
    private final SimVisionSystem m_simCamera;
    private static boolean m_instanceExists = false;

    public final Consumer<Pose2d> updatePose;

    private SimulatedCamera() {
        m_simCamera = new SimVisionSystem(
                VisionConstants.CAMERA_NAME,
                VisionConstants.CAMERA_FOV_DEGREES,
                Vision.getCameraTransform(),
                VisionConstants.MAX_LED_RANGE_METERS,
                VisionConstants.CAMERA_RESOLUTION_HEIGHT,
                VisionConstants.CAMERA_RESOLUTION_HEIGHT,
                m_minTargetArea);

        updatePose = (pose) -> m_simCamera.processFrame(pose);
    }

    public void addVisionTargets(AprilTagFieldLayout layout) {
        m_simCamera.addVisionTargets(layout);
    }

    public static SimulatedCamera createInstance() {
        if (!m_instanceExists) {
            m_instanceExists = true;
            return new SimulatedCamera();
        } else
            throw new RuntimeException("Simulated Camera already referenced");
    }

}