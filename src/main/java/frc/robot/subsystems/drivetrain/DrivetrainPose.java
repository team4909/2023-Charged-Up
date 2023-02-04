package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.camera.CameraBase;
import frc.robot.subsystems.vision.camera.LimelightCamera;

public class DrivetrainPose extends SubsystemBase {

    CameraBase camera = new LimelightCamera();

    /* Singleton */
    private static DrivetrainPose m_instance = null;

    public static DrivetrainPose getInstance() {
        if (m_instance == null) {
            m_instance = new DrivetrainPose();
        }
        return m_instance;
    }

    public Vision pv;
    public Optional<EstimatedRobotPose> robotPose;

    public DrivetrainPose() {
        pv = new Vision();
        robotPose = pv.getEstimatedRobotPose();
    }

    public void updateRobotPose() {
        robotPose = pv.getEstimatedRobotPose();
    }

    int num = 0;

    @Override
    public void periodic() {
        if (num == 100) {
            updateRobotPose();
            System.out.println("updated robot pose");
            num = 0;
        } else {
            num++;
        }
        if (robotPose.isPresent())
            SmartDashboard.putString("Pose", robotPose.get().estimatedPose.toString());
    }
}