package frc.robot.subsystems.drivetrain.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;

public class TrajectoryFollow extends CommandBase {

    private String m_pathName;
    private PathPlannerTrajectory m_trajectory;
    private double m_timeout = DrivetrainConstants.DEFAULT_TIMEOUT;
    private final Timer timer = new Timer();
    private final Drivetrain m_drivetrain = Drivetrain.getInstance();

    public TrajectoryFollow() {
        m_pathName = "Stay Still";
    }

    public TrajectoryFollow(String pathName) {
        m_pathName = pathName;
    }

    public TrajectoryFollow(Pair<String, Double> traj) {
        m_pathName = traj.getFirst();
        m_timeout = traj.getSecond();
    }

    @Override
    public void initialize() {
        timer.start();
        m_drivetrain.setState(DrivetrainStates.AUTONOMOUS);
        m_trajectory = PathPlanner.loadPath(m_pathName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED, 3);
        if (m_trajectory == null) {
            DriverStation.reportError("Path not loaded correctly!", Thread.currentThread().getStackTrace());
            return;
        }

        m_drivetrain.setFieldTrajectory(m_trajectory);

        new PPSwerveControllerCommand(
                m_trajectory,
                m_drivetrain.m_poseSupplier,
                m_drivetrain.getKinematics(),
                new PIDController(6, 0, 0),
                new PIDController(6, 0, 0),
                new PIDController(6, 0, 0),
                m_drivetrain.m_swerveModuleConsumer,
                true,
                m_drivetrain)
                .withTimeout(m_timeout).schedule(); 
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setState(DrivetrainStates.JOYSTICK_DRIVE); // TODO change this to idle
    }
}
