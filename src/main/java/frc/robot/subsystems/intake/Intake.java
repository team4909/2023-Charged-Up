package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private static Intake m_instance;
  private IntakeStates m_state, m_lastState;

  private final CANSparkMax m_pivotRight, m_pivotLeft, m_frontRoller, m_backRoller;
  private double m_hingeSetpoint;

  public enum IntakeStates {
    IDLE("Idle"),
    RETRACTED("Retracted"),
    CALIBRATE("Calibrate"),
    INTAKE_CUBE("Intake Cube"),
    INTAKE_CONE("Intake Cone"),
    SPIT_CONE("Spit Cone"),
    SPIT_CUBE("Spit Cube"),
    HANDOFF("Handoff");

    String stateName;

    private IntakeStates(String name) {
      this.stateName = name;
    }

    public boolean was(IntakeStates state) {
      return this.equals(state);
    }

    public String toString() {
      return stateName;
    }
  }

  private Intake() {
    m_state = IntakeStates.IDLE;
    m_pivotRight = new CANSparkMax(IntakeConstants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);
    m_pivotLeft = new CANSparkMax(IntakeConstants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    m_frontRoller = new CANSparkMax(IntakeConstants.FRONT_ROLLER_MOTOR, MotorType.kBrushless);
    m_backRoller = new CANSparkMax(IntakeConstants.BACK_ROLLER_MOTOR, MotorType.kBrushless);

    m_pivotLeft.restoreFactoryDefaults();
    m_pivotRight.restoreFactoryDefaults();
    m_frontRoller.restoreFactoryDefaults();
    m_backRoller.restoreFactoryDefaults();

    m_pivotRight.follow(m_pivotLeft);

    m_pivotLeft.getPIDController().setP(IntakeConstants.kP);
    m_pivotLeft.getPIDController().setOutputRange(-IntakeConstants.OUTPUT_LIMIT, IntakeConstants.OUTPUT_LIMIT);

    m_pivotLeft.setSmartCurrentLimit(40, 40);
    m_pivotLeft.getEncoder().setPositionConversionFactor(IntakeConstants.DEGREES_PER_TICK);
    m_pivotLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putNumber("Pivot error", m_pivotLeft.getEncoder().getPosition() - m_hingeSetpoint);
    SmartDashboard.putNumber("Pivot Setpoint", m_hingeSetpoint);
    SmartDashboard.putNumber("Pivot Encoder Position", m_pivotLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("Pivot output", m_pivotLeft.getAppliedOutput());
    SmartDashboard.putString("Intake State", m_state.toString());
    SmartDashboard.putNumber("Pivot Current", m_pivotLeft.getOutputCurrent());
  }

  private void stateMachine() {
    Command currentIntakeCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentIntakeCommand = Idle();
          break;
        case RETRACTED:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.RETRACTED_SETPOINT, 0d, 0d);
          break;
        case INTAKE_CUBE:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.CUBE_SETPOINT, 0.5, -0.5);
          break;
        case INTAKE_CONE:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.CONE_SETPOINT, 0.375, -0.75);
          break;
        case SPIT_CUBE:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.CUBE_SETPOINT, -0.75, 0.75);
          break;
        case SPIT_CONE:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.RETRACTED_SETPOINT, 0.75d, -0.75d);
          break;
        case CALIBRATE:
          currentIntakeCommand = Calibrate();
          break;
        case HANDOFF:
          if (m_lastState.was(IntakeStates.INTAKE_CUBE)) {
            IntakeStates.HANDOFF.stateName.concat(" Cube");
            currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.HANDOFF_SETPOINT, 0.2d, -0.05d);
          } else if (m_lastState.was(IntakeStates.INTAKE_CONE)) {
            IntakeStates.HANDOFF.stateName.concat(" Cone");
            currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.HANDOFF_SETPOINT, 0.2d, 0.25d);
          }
          break;
        default:
          m_state = IntakeStates.IDLE;
          break;
      }

      m_lastState = m_state;

      if (currentIntakeCommand != null) {
        currentIntakeCommand.schedule();
      }
    }

  }

  // #region State Commands
  private Command Idle() {
    return Commands.runOnce(() -> {
      m_pivotLeft.set(0d);
      m_frontRoller.set(0d);
      m_backRoller.set(0d);
    }, this);
  }

  private Command SetPivotPositionAndRollerSpeed(double position, double frontSpeed, double backSpeed) {
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_frontRoller.set(frontSpeed);
          m_backRoller.set(backSpeed);
        }, this),
        Commands.run(() -> {
          var arbFF = MathUtil.clamp(calcFF(m_pivotLeft.getEncoder().getPosition()), -1d, 1d);
          m_hingeSetpoint = position; // For Telemetry purposes
          m_pivotLeft.getPIDController().setReference(m_hingeSetpoint, ControlType.kPosition, 0,
              arbFF);
        }, this));
  }

  private Command Calibrate() {
    return Commands.run(() -> {
      m_pivotLeft.setSmartCurrentLimit(5, 40);
      m_pivotLeft.set(0.1);
    }, this).withTimeout(0.75)
        .andThen(Commands.run(() -> {
          m_pivotLeft.getEncoder().setPosition(110d);
          m_state = IntakeStates.RETRACTED;
          m_pivotLeft.setSmartCurrentLimit(40, 40);
        }, this));
  }
  // #endregion

  private double calcFF(double theta) {
    double ff = IntakeConstants.kG * Math.cos(Math.toRadians(theta));
    SmartDashboard.putNumber("Intake FF", ff);
    return ff;
  }

  public Command setState(IntakeStates state) {
    return Commands.runOnce(() -> m_state = state);
  }

  public static Intake getInstance() {
    return m_instance = (m_instance == null) ? new Intake() : m_instance;
  }
}
