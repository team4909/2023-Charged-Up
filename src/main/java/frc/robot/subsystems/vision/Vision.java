package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

  private final NetworkTableInstance NT = NetworkTableInstance.getDefault();

  private Optional<LimelightResults> visionResults() {
    if (NT.getTable("limelight").getKeys().size() != 0
        && NT.getTable("limelight").getEntry("tv").getInteger(0) == 1)
      return Optional.of(LimelightHelpers.getLatestResults("limelight"));
    return Optional.empty();
  }

  public Supplier<Double> latency = () -> visionResults().isPresent()
      ? Timer.getFPGATimestamp() - visionResults().get().targetingResults.botpose[6] / 1000
      : null;

  public Pose2d getAllianceRelativePose() {
    if (visionResults().isPresent()) {
      if (DriverStation.getAlliance().equals(Alliance.Red))
        visionResults().get().targetingResults.getBotPose2d_wpiRed();
      else if (DriverStation.getAlliance().equals(Alliance.Blue))
        visionResults().get().targetingResults.getBotPose2d_wpiBlue();
    }
    return null;
  }

  public PathPlannerTrajectory generateOnTheFlyTrajectory(double drivetrainVelocity) {
    PathPlannerTrajectory onTheFlyTrajectory = PathPlanner.generatePath(
        new PathConstraints(4, 3),
        new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)));
    return onTheFlyTrajectory;
  }
}
