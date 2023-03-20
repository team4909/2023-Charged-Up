package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;
import frc.lib.limelight.LimelightHelpers.Results;

public class Vision extends SubsystemBase {

  private final NetworkTableInstance NT;
  private final AprilTagFieldLayout m_aprilTagFieldLayout;

  private final int[] kBlueTags = new int[] { 4, 6, 7, 8 };
  private final int[] kRedTags = new int[] { 5, 3, 2, 1 };
  private final double kDistTagToCone = Units.inchesToMeters(21.245);
  private final double kDistTagToTape = Units.inchesToMeters(30.25);
  private final double kTagToSingleSubstationX = Units.inchesToMeters(60.467);
  private final double kTagToSingleSubstationY = Units.inchesToMeters(36.74);

  private Results m_results;

  public Vision() {
    NT = NetworkTableInstance.getDefault();
    m_aprilTagFieldLayout = loadFieldLayout();
  }

  private AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  private Optional<LimelightResults> visionResults() {
    if (NT.getTable("limelight").getKeys().size() != 0
        && NT.getTable("limelight").getEntry("tv").getDouble(0) == 1)
      return Optional.of(LimelightHelpers.getLatestResults("limelight"));
    return Optional.empty();
  }

  public Supplier<Double> latency = () -> visionResults().isPresent()
      ? Timer.getFPGATimestamp() - (m_results.latency_pipeline / 1000.0) - (m_results.latency_capture / 1000.0)
      : null;

  public Pose2d getAllianceRelativePose() {
    visionResults().ifPresent((results) -> m_results = results.targetingResults);
    if (m_results != null) {
      if (DriverStation.getAlliance().equals(Alliance.Red))
        return m_results.getBotPose2d_wpiRed();
      else if (DriverStation.getAlliance().equals(Alliance.Blue))
        return m_results.getBotPose2d_wpiBlue();
    }
    return null;
  }

  /**
   * 
   * @param currentPose
   * @param currentSpeed
   * @param node         Top to bottom ascending order, [single substation: 0,
   *                     double substation: 1, first cone node: 2, first cube
   *                     node: 3, etc...]
   * @return Trajectory with points relative to the alliance origin
   */
  public PathPlannerTrajectory generateOnTheFlyTrajectory(Pose2d currentPose, ChassisSpeeds currentSpeed, int node) {

    final int[] tags;
    if (DriverStation.getAlliance().equals(Alliance.Blue))
      tags = kBlueTags;
    else if (DriverStation.getAlliance().equals(Alliance.Red))
      tags = kRedTags;
    else
      tags = null;

    final Pose2d goalPose;
    switch (node) {
      case 0:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), new Rotation2d()));
        break;
      case 1:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[0]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kTagToSingleSubstationX, kTagToSingleSubstationY),
                Rotation2d.fromDegrees(-90d)));
        break;
      case 2:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), new Rotation2d()));
        break;
      case 3:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), new Rotation2d()));
        break;
      case 4:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), new Rotation2d()));
        break;
      case 5:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), new Rotation2d()));
        break;
      case 6:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), new Rotation2d()));
        break;
      case 7:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), new Rotation2d()));
        break;
      case 8:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), new Rotation2d()));
        break;
      case 9:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), new Rotation2d()));
        break;
      case 10:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), new Rotation2d()));
        break;
      default:
        goalPose = currentPose;
        break;
    }

    PathPlannerTrajectory onTheFlyTrajectory = PathPlanner.generatePath(
        new PathConstraints(4, 3),
        PathPoint.fromCurrentHolonomicState(currentPose, currentSpeed),
        new PathPoint(goalPose.getTranslation(), Rotation2d.fromDegrees(180),
            goalPose.getRotation().plus(Rotation2d.fromDegrees(-180))));
    return onTheFlyTrajectory;
  }
}
