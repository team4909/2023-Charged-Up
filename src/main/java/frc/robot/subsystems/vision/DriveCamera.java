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

    private final UsbCamera m_camera;
    private final CvSink m_inputStream;
    private final CvSource m_outputStream;

    private Mat m_processFrame;

    public DriveCamera() {
        m_camera = CameraServer.startAutomaticCapture(1);
        m_camera.setResolution(240, 180);
        m_camera.setFPS(30);
        m_inputStream = CameraServer.getVideo();
        m_outputStream = CameraServer.putVideo("Crosshair", 240, 180);
    }

    public void startPipeline() {
        m_inputStream.grabFrame(m_processFrame);
        double width = m_processFrame.width();
        double height = m_processFrame.height();
        Imgproc.line(m_processFrame, new Point(0, 0), new Point(width, height), new Scalar(0.3), 5);
        Imgproc.line(m_processFrame, new Point(width, 0), new Point(0, height), new Scalar(0.3), 5);
        m_outputStream.putFrame(m_processFrame);
    }

}
