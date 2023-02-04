package frc.robot.subsystems.vision.camera;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public final class SimulatedCamera extends CameraBase {

    private final double m_minTargetArea = 10d;
    private final SimVisionSystem m_simCamera;

    public SimulatedCamera() {
        Transform3d robotToCamera = new Transform3d(
                new Translation3d(0d, VisionConstants.CAMERA_HEIGHT_METERS, 0d),
                new Rotation3d(0d, VisionConstants.CAMERA_PITCH_RADIANS, 0d));

        m_simCamera = new SimVisionSystem(
                VisionConstants.CAMERA_NAME,
                VisionConstants.CAMERA_FOV_DEGREES,
                robotToCamera,
                VisionConstants.MAX_LED_RANGE_METERS,
                VisionConstants.CAMERA_RESOLUTION_HEIGHT,
                VisionConstants.CAMERA_RESOLUTION_HEIGHT,
                m_minTargetArea);

        m_simCamera.addVisionTargets(super.m_aprilTagFieldLayout);
    }

    @Override
    void update() {
        m_simCamera.processFrame(new Pose2d());
    }

}