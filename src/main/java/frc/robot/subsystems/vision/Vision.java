package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;
import frc.lib.limelight.LimelightHelpers.Results;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

  private final NetworkTableInstance NT;

  private Results m_results;

  public Vision() {
    NT = NetworkTableInstance.getDefault();

    LimelightHelpers.setLEDMode_ForceOff("limelight");
    LimelightHelpers.setCameraPose_RobotSpace("limelight",
        VisionConstants.ROBOT_TO_CAM.getX(),
        VisionConstants.ROBOT_TO_CAM.getY(),
        VisionConstants.ROBOT_TO_CAM.getZ(),
        VisionConstants.ROBOT_TO_CAM.getRotation().getX(),
        VisionConstants.ROBOT_TO_CAM.getRotation().getY(),
        VisionConstants.ROBOT_TO_CAM.getRotation().getZ());
  }

  private Optional<LimelightResults> visionResults() {
    if (NT.getTable("limelight").getKeys().size() != 0
        && NT.getTable("limelight").getEntry("tv").getDouble(0) == 1)
      return Optional.of(LimelightHelpers.getLatestResults("limelight"));
    return Optional.empty();
  }

  public Pair<Pose2d, Double> getAllianceRelativePose() {
    visionResults().ifPresentOrElse(((results) -> m_results = results.targetingResults), () -> m_results = null);
    Pair<Pose2d, Double> val = new Pair<>(null, null);
    if (m_results != null) {
      double latency = Timer.getFPGATimestamp()
          - (m_results.latency_pipeline / 1000.0)
          - (m_results.latency_capture / 1000.0);
      if (DriverStation.getAlliance().equals(Alliance.Red))
        val = Pair.of(m_results.getBotPose2d_wpiRed(), latency);
      else if (DriverStation.getAlliance().equals(Alliance.Blue))
        val = Pair.of(m_results.getBotPose2d_wpiBlue(), latency);
    }
    return val;
  }

}
