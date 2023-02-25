package frc.robot.subsystems.drivetrain.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveTrajectory {
    private final String m_pathName;
    private final PathPlannerTrajectory m_trajectory;
    private final double m_timeout;
    private final Drivetrain m_drivetrain = Drivetrain.getInstance();
    private final boolean m_isFirstPath;

    public DriveTrajectory(String pathName, boolean isFirstPath) {
        this(new Pair<String, Double>(pathName, DrivetrainConstants.DEFAULT_TIMEOUT), isFirstPath);
    }

    public DriveTrajectory(Pair<String, Double> traj, boolean isFirstPath) {
        m_pathName = traj.getFirst();
        m_timeout = traj.getSecond();
        m_isFirstPath = isFirstPath;
        m_trajectory = PathPlanner.loadPath(m_pathName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED / 10, 3);
        if (m_trajectory == null) {
            DriverStation.reportError("Path not loaded correctly!", Thread.currentThread().getStackTrace());
            return;
        }

        if (m_isFirstPath)
            m_drivetrain.resetPose(m_trajectory.getInitialHolonomicPose());
    }

    public PathPlannerTrajectory getTrajectory() {
        return m_trajectory;
    }

    public double getTimeout() {
        return m_timeout;
    }
}
