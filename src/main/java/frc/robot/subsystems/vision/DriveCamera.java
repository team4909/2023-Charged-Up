package frc.robot.subsystems.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class DriveCamera {

    public DriveCamera() {
        Thread cameraThread = new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);
            camera.setResolution(240, 180);
            camera.setFPS(20);

            CvSink cvSink = CameraServer.getVideo(camera);
            CvSource outputStream = CameraServer.putVideo("Crosshair", 240, 180);

            Mat source = new Mat();
            Mat output = new Mat();

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                double width = output.width();
                double height = output.height();
                Imgproc.line(output, new Point(0, 0), new Point(width, height), new Scalar(0.3), 5);
                Imgproc.line(output, new Point(width, 0), new Point(0, height), new Scalar(0.3), 5);
                outputStream.putFrame(output);
            }
        });
        cameraThread.setName("Camera");
        cameraThread.start();
    }
}
