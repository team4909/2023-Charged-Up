package frc.robot.subsystems.drivetrain.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.DrivetrainConstants;

public record DriveTrajectory(PathPlannerTrajectory trajectory, boolean isFirstPath) {
  public DriveTrajectory(String trajectoryName, boolean isFirstPath) {
    this(
        PathPlanner.loadPath(trajectoryName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED, 1.2),
        isFirstPath);
  }

  public DriveTrajectory(String trajectoryName, boolean isFirstPath, double maxVel, double maxAccel) {
    this(
        PathPlanner.loadPath(trajectoryName, maxVel, maxAccel),
        isFirstPath);
  }

  public DriveTrajectory(String trajectoryName, boolean isFirstPath, double maxAccel) {
    this(
        PathPlanner.loadPath(trajectoryName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED, maxAccel),
        isFirstPath);
  }
}
