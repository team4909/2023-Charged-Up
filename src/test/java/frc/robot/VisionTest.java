package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.vision.camera.CameraBase;
import frc.robot.subsystems.vision.camera.SimulatedCamera;

public class VisionTest {

    SwerveDrivePoseEstimator m_swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
    CameraBase camera = new SimulatedCamera();

    {

    }

}
