package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    public Supplier<Pose2d> robotPose = () -> visionResults().isPresent()
            ? visionResults().get().targetingResults.getBotPose2d()
            : null;

    public Supplier<Double> latency = () -> visionResults().isPresent()
            ? Timer.getFPGATimestamp() - visionResults().get().targetingResults.botpose[6] / 1000
            : null;
}
