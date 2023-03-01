package frc.robot.subsystems.drivetrain.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrajectory {
    private final String m_pathName;
    private final PathPlannerTrajectory m_trajectory;
    private final double m_timeout;
    private final boolean m_isFirstPath;

    public DriveTrajectory(String pathName, boolean isFirstPath) {
        this(new Pair<String, Double>(pathName, DrivetrainConstants.DEFAULT_TIMEOUT), isFirstPath);
    }

    public DriveTrajectory(Pair<String, Double> trajectory, boolean isFirstPath) {
        m_pathName = trajectory.getFirst();
        m_timeout = trajectory.getSecond();
        m_isFirstPath = isFirstPath;
        m_trajectory = PathPlanner.loadPath(m_pathName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED, 3);
        if (m_trajectory == null) {
            DriverStation.reportError("Path not loaded correctly!", Thread.currentThread().getStackTrace());
            return;
        }
    }

    public PathPlannerTrajectory getTrajectory() {
        return m_trajectory;
    }

    public double getTimeout() {
        return m_timeout;
    }

    public boolean getIsFirstPath() {
        return m_isFirstPath;
    }
}
