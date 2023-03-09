package frc.robot.subsystems.drivetrain.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.DrivetrainConstants;

public record DriveTrajectory(PathPlannerTrajectory trajectory, boolean isFirstPath) {
  public DriveTrajectory(String trajectoryName, boolean isFirstPath) {
    this(
        PathPlanner.loadPath(trajectoryName, DrivetrainConstants.MAX_DRIVETRAIN_SPEED, 3),
        isFirstPath);
  }
}
