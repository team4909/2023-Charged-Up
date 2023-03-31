package frc.robot.subsystems.vision;

import java.io.IOException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionUtils {
  private final AprilTagFieldLayout m_aprilTagFieldLayout;

  private final int[] kBlueTags = new int[] { 4, 6, 7, 8 };
  private final int[] kRedTags = new int[] { 5, 3, 2, 1 };
  private final double kDistTagToCone = Units.inchesToMeters(21.245);
  private final double kDistTagToTape = Units.inchesToMeters(30.25);
  private final double kTagToSingleSubstationX = Units.inchesToMeters(60.467);
  private final double kTagToSingleSubstationY = Units.inchesToMeters(36.74);

  public VisionUtils() {
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
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      tags = kBlueTags;
      m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
      tags = kRedTags;
      m_aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    } else
      tags = null;

    final Pose2d goalPose;
    switch (node) {
      case 0:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 1:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[0]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kTagToSingleSubstationX, kTagToSingleSubstationY),
                Rotation2d.fromDegrees(-90d)));
        break;
      case 2:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 3:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), Rotation2d.fromDegrees(-180.0)));
        break;
      case 4:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[1]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 5:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 6:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), Rotation2d.fromDegrees(-180.0)));
        break;
      case 7:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[2]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, -kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 8:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, kDistTagToCone), Rotation2d.fromDegrees(-180.0)));
        break;
      case 9:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), Rotation2d.fromDegrees(-180.0)));
        break;
      case 10:
        goalPose = m_aprilTagFieldLayout.getTagPose(tags[3]).get().toPose2d()
            .plus(new Transform2d(new Translation2d(kDistTagToTape, 0), Rotation2d.fromDegrees(-180.0)));
        break;
      default:
        goalPose = currentPose;
        break;
    }

    PathPoint endPoint = new PathPoint(goalPose.getTranslation(), Rotation2d.fromDegrees(180), goalPose.getRotation());

    PathPlannerTrajectory onTheFlyTrajectory = PathPlanner.generatePath(
        new PathConstraints(4, 3),
        PathPoint.fromCurrentHolonomicState(currentPose, currentSpeed),
        endPoint);
    return onTheFlyTrajectory;
  }
}
