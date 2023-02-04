package frc.robot.subsystems.vision.camera;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public abstract class CameraBase {

    public double yawOffset;

    protected final AprilTagFieldLayout m_aprilTagFieldLayout = loadLayout();

    abstract void update();

    private AprilTagFieldLayout loadLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        return m_aprilTagFieldLayout;
    }
}