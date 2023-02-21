package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class DriveCamera {

    public DriveCamera() {
        Thread cameraThread = new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);
            camera.setResolution(240, 180);
            camera.setFPS(20);
        });
        cameraThread.setName("Camera");
        cameraThread.start();
    }
}
