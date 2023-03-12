package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;
import frc.robot.Constants;

public class Vision {

    private final NetworkTableInstance NT = NetworkTableInstance.getDefault();

    private Optional<LimelightResults> visionResults() {
        if (NT.getTable("limelight").getKeys().size() != 0
                && NT.getTable("limelight").getEntry("tv").getInteger(0) == 1)
            return Optional.of(LimelightHelpers.getLatestResults("limelight"));
        return Optional.empty();
    }

    public Supplier<Pose2d> robotPose = () -> getAllianceRelativePose();

    public Supplier<Double> latency = () -> visionResults().isPresent()
            ? Timer.getFPGATimestamp() - visionResults().get().targetingResults.botpose[6] / 1000
            : null;

    public DoubleSupplier xOffset = () -> getOffsetDistance();
    public IntConsumer changePipeline = p -> NT.getTable("limelight").getEntry("pipeline").setNumber(p);

    private Pose2d getAllianceRelativePose() {
        if (visionResults().isPresent()) {
            if (Constants.ALLIANCE.equals(Alliance.Red))
                visionResults().get().targetingResults.getBotPose2d_wpiRed();
            else if (Constants.ALLIANCE.equals(Alliance.Blue))
                visionResults().get().targetingResults.getBotPose2d_wpiBlue();
        }
        return null;
    }

    private double getOffsetDistance() {
        if (visionResults().isPresent()) {
            SmartDashboard.putNumber("Vision/April Tag distance", LimelightHelpers.getTX("limelight"));
            return LimelightHelpers.getTX("limelight");
        }
        return 0d;
    }

}
