package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Arm extends SubsystemBase {
    private static Arm m_instance;
    private ArmStates m_state, m_lastState;

    private final CANSparkMax m_wristMotor;
    private final ArmFeedforward m_armFeedForward;

    public enum ArmStates {
        IDLE("Idle"),
        ZERO("Zero"),
        TOP("Top"),
        HANDOFF_CONE("Handoff Cone"),
        HANDOFF_CUBE("Handoff Cube"),
        RETRACTED("Retracted"),
        DROPPING("Dropping"),
        DROPPING_FLICK("Dropping Flick");

        String stateName;

        private ArmStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return this.stateName;
        }
    }

    private Arm() {
        m_state = ArmStates.IDLE;
        m_wristMotor = new CANSparkMax(6, MotorType.kBrushless);
        m_armFeedForward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV,
                WristConstants.kA);
        configureHardware();
    }

    public void periodic() {
        stateMachine();
        SmartDashboard.putString("state", m_state.toString());
        SmartDashboard.putNumber("wrist encoder pos deg", getWristEncoderPos());
        SmartDashboard.putNumber("UNdo conversion",
                getWristEncoderPos() / WristConstants.DEGREES_PER_TICK);
        SmartDashboard.putNumber("Velocity (ticks)", m_wristMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("smart motion allowed close loop error",
                m_wristMotor.getPIDController().getSmartMotionAllowedClosedLoopError(0));
        SmartDashboard.putNumber("Output", m_wristMotor.get());
    }

    private void stateMachine() {
        Command currentWristCommand = null;
        if (!m_state.equals(m_lastState)) {
            switch (m_state) {
                case IDLE:
                    currentWristCommand = Idle();
                    break;
                case ZERO:
                    currentWristCommand = Zero();
                    break;
                case TOP:
                    currentWristCommand = SetWristPosition(110);
                    break;
                case HANDOFF_CONE:
                    currentWristCommand = SetWristPosition(-40.361);
                    break;
                case HANDOFF_CUBE:
                    currentWristCommand = SetWristPosition(-7.966);
                    break;
                case DROPPING: 
                    currentWristCommand = SetWristPosition(0);
                    break;
                case DROPPING_FLICK:
                    currentWristCommand = SetWristPosition(-40);
                    break;
                case RETRACTED:
                    currentWristCommand = Retract();
                    break;
                default:
                    m_state = ArmStates.IDLE;
            }
        }

        m_lastState = m_state;

        if (currentWristCommand != null) {
            currentWristCommand.schedule();
        }
    }

    private Command Idle() {
        return new InstantCommand(() -> {
        }, this).repeatedly().withTimeout(WristConstants.ZERO_TIME)
                .andThen(() -> {
                    // m_clawMotor.getEncoder().setPosition(0d);
                    // m_wristMotor.getEncoder().setPosition(0d);
                }, this);
    }

    private Command Zero() {
        return new InstantCommand(() -> {
            // m_wristMotor.getEncoder().setPosition(0);
        }, this);
    }

    private Command SetWristPosition(double setpoint) {
        return new InstantCommand(() -> {
            setWristSetpoint(setpoint);
        }, this);
    }

    private Command Retract() {
        return new InstantCommand(() -> {
        }, this);
    }

    private void setWristSetpoint(double setpoint) {
        double arbFF = m_armFeedForward.calculate(getWristEncoderPos(),
                Units.rotationsPerMinuteToRadiansPerSecond(m_wristMotor.getEncoder().getVelocity()));
        // m_wristMotor.getPIDController().setReference(setpoint,
        // ControlType.kSmartMotion, 0, arbFF);
        m_wristMotor.getPIDController().setReference(setpoint, ControlType.kPosition);
        // m_wristMotor.getPIDController().setReference(setpoint,
        // ControlType.kSmartMotion);
    }

    public void setState(ArmStates state) {
        m_state = state;
    }

    private void configureHardware() {

        m_wristMotor.restoreFactoryDefaults();
        m_wristMotor.getPIDController().setP(WristConstants.kP);
        m_wristMotor.getPIDController().setFF(0.005);
        m_wristMotor.getPIDController().setOutputRange(-WristConstants.OUTPUT_LIMIT, WristConstants.OUTPUT_LIMIT);
        m_wristMotor.setIdleMode(IdleMode.kBrake);
        // m_wristMotor.getPIDController().setSmartMotionMaxVelocity(2, 0);
        // m_wristMotor.getPIDController().setSmartMotionMaxAccel(1, 0);
        // m_wristMotor.getEncoder().setPosition(0);
        m_wristMotor.getEncoder().setPositionConversionFactor(WristConstants.DEGREES_PER_TICK);
        m_wristMotor.setInverted(true);

    }

    private double getWristEncoderPos() {
        return m_wristMotor.getEncoder().getPosition();
    }

    public static Arm getInstance() {
        if (m_instance == null) {
            m_instance = new Arm();
        }
        return m_instance;

    }

}