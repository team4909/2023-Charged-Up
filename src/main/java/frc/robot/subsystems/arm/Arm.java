package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Arm extends SubsystemBase {
    private static Arm m_instance;
    private ArmStates m_state, m_lastState;

    private final CANSparkMax m_wristMotor;
    private final ArmFeedforward m_armFeedForward;
    private double m_arbFF;

    public enum ArmStates {
        IDLE("Idle"),
        RETRACTED("Top"),
        HANDOFF_CONE("Handoff Cone"),
        HANDOFF_CUBE("Handoff Cube"),
        DROPPING("Dropping");

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
        SmartDashboard.putNumber("Wrist Output", m_wristMotor.getAppliedOutput());
        SmartDashboard.putNumber("Arb FF", m_arbFF);
    }

    private void stateMachine() {
        Command currentWristCommand = null;
        if (!m_state.equals(m_lastState)) {
            switch (m_state) {
                case IDLE:
                    currentWristCommand = Idle();
                    break;
                case RETRACTED:
                    currentWristCommand = SetWristPosition(110);
                    break;
                case HANDOFF_CONE:
                    currentWristCommand = SetWristPosition(-47.361);
                    break;
                case HANDOFF_CUBE:
                    currentWristCommand = SetWristPosition(-7.966);
                    break;
                case DROPPING:
                    currentWristCommand = SetWristPosition(0);
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
            m_wristMotor.set(0);
        }, this);
    }

    private Command SetWristPosition(double setpoint) {
        return new RunCommand(() -> {
            setWristSetpoint(setpoint);
            SmartDashboard.putNumber("Wrist Setpoint", setpoint);
        }, this);
    }

    private void setWristSetpoint(double setpoint) {
        // m_arbFF = m_armFeedForward.calculate(getWristEncoderPos(),
        // Units.rotationsPerMinuteToRadiansPerSecond(m_wristMotor.getEncoder().getVelocity())
        // * (180d / Math.PI));
        // m_wristMotor.getPIDController().setReference(setpoint,
        // ControlType.kSmartMotion, 0, m_arbFF);
        // m_wristMotor.getPIDController().setReference(setpoint,
        // ControlType.kPosition);
        // double ff = -calcFF(m_wristMotor.getEncoder().getPosition());
        m_wristMotor.getPIDController().setReference(setpoint, ControlType.kPosition, 0,
                calcFF(Units.degreesToRadians(m_wristMotor.getEncoder().getPosition()),
                        Units.rotationsPerMinuteToRadiansPerSecond(m_wristMotor.getEncoder().getVelocity())));
    }

    public void setState(ArmStates state) {
        m_state = state;
    }

    public Command setState2(ArmStates state) {
        return new InstantCommand(() -> m_state = state);
    }

    private void configureHardware() {

        m_wristMotor.restoreFactoryDefaults();
        m_wristMotor.setInverted(true);
        m_wristMotor.getPIDController().setP(WristConstants.kP);
        m_wristMotor.getPIDController().setOutputRange(-WristConstants.OUTPUT_LIMIT, WristConstants.OUTPUT_LIMIT);
        m_wristMotor.setIdleMode(IdleMode.kBrake);

        m_wristMotor.getPIDController().setSmartMotionAllowedClosedLoopError(200, 0);

        m_wristMotor.getEncoder().setPositionConversionFactor(WristConstants.DEGREES_PER_TICK);
        m_wristMotor.getEncoder().setPosition(121);

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

    private double calcFF(double theta, double vel) {
        // double ff = m_armFeedForward.calculate(theta, vel);
        double ff = WristConstants.kG * Math.cos(theta);
        SmartDashboard.putNumber("Intake FF", ff);
        return MathUtil.clamp(ff, -1d, 1d);
    }

}