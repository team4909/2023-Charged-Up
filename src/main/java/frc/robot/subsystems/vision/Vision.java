package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

    private final NetworkTableInstance NT = NetworkTableInstance.getDefault();

    private static Vision m_instance = null;
    private LimelightResults m_visionResults;

    private VisionStates m_state = VisionStates.IDLE;
    private VisionStates m_lastState;

    final double REACH_DISTANCE = 5d;

    public enum VisionStates {
        IDLE("Idle"),
        LOCALIZING("Localizing");

        String stateName;

        private VisionStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return this.stateName;
        }
    }

    private Vision() {
        NetworkTableInstance.create().close();
    }

    @Override
    public void periodic() {
        stateMachine();
        if (NetworkTableInstance.getDefault().getTable("limelight").getKeys().size() != 0)
            m_visionResults = LimelightHelpers.getLatestResults("limelight");
    }

    public Optional<LimelightResults> getVisionResults() {
        final NetworkTableInstance NT = NetworkTableInstance.getDefault();
        if (NT.getTable("limelight").getKeys().size() != 0
                && NT.getTable("limelight").getEntry("tv").getInteger(0) == 1)
            return Optional.of(LimelightHelpers.getLatestResults("limelight"));
        return Optional.empty();
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

    private double getThetaToTurn() {
        var xError = 0;
        return Math.acos(xError / REACH_DISTANCE);
    }

    private void stateMachine() {
        Command currentVisionCommand = null;
        if (!m_state.equals(m_lastState)) {
            switch (m_state) {
                case IDLE:
                    currentVisionCommand = new InstantCommand();
                    break;
                case LOCALIZING:
                    currentVisionCommand = new InstantCommand();
                    break;
                default:
                    m_state = VisionStates.IDLE;
                    break;
            }
        }

        m_lastState = m_state;

        if (currentVisionCommand != null) {
            currentVisionCommand.schedule();
        }
    }

}
