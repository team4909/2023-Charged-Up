package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CubeShooterConstants;

public class CubeShooter extends SubsystemBase {
  private static CubeShooter m_instance;
  private CubeShooterStates m_state, m_lastState;

  private final CANSparkMax m_cubePivot, m_topRoller, m_bottomRoller;
  private double m_cubePivotSetpoint;

  public enum CubeShooterStates {
    IDLE("Idle"),
    CUBE_DOWN("Down"),
    CUBE_UP("Up"),
    CUBE_MID("Cube Mid"),
    CUBE_HIGH("Cube High"),
    CALIBRATE("Cube Calibrate"),
    CUBE_SPIT("Cube Spit"),
    CUBE_SPIT_HIGH("Cube Spit High");

    String stateName;

    private CubeShooterStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return stateName;
    }
  }

  private CubeShooter() {
    m_state = CubeShooterStates.IDLE;
    m_cubePivot = new CANSparkMax(CubeShooterConstants.PIVOT_MOTOR, MotorType.kBrushless);
    m_topRoller = new CANSparkMax(CubeShooterConstants.TOP_ROLLER_MOTOR, MotorType.kBrushless);
    m_bottomRoller = new CANSparkMax(CubeShooterConstants.BOTTOM_ROLLER_MOTOR, MotorType.kBrushless);

    m_cubePivot.restoreFactoryDefaults();
    m_topRoller.restoreFactoryDefaults();
    m_bottomRoller.restoreFactoryDefaults();

    m_cubePivot.getPIDController().setP(CubeShooterConstants.kP);
    m_cubePivot.getPIDController().setD(CubeShooterConstants.kD);
    m_cubePivot.getPIDController().setOutputRange(-CubeShooterConstants.OUTPUT_LIMIT,
        CubeShooterConstants.OUTPUT_LIMIT);

    m_cubePivot.setSmartCurrentLimit(40, 40);
    m_cubePivot.getEncoder().setPositionConversionFactor(CubeShooterConstants.DEGREES_PER_TICK);
    m_cubePivot.setInverted(false);

  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putNumber("Cube Shooter/Pivot Error",
        m_cubePivot.getEncoder().getPosition() - m_cubePivotSetpoint);
    SmartDashboard.putNumber("Cube Shooter/Pivot Setpoint", m_cubePivotSetpoint);
    SmartDashboard.putNumber("Cube Shooter/Pivot Encoder Position", m_cubePivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Cube Shooter/Pivot Output", m_cubePivot.getAppliedOutput());
    SmartDashboard.putNumber("Cube Shooter/Pivot Current", m_cubePivot.getOutputCurrent());
    SmartDashboard.putString("Cube Shooter/State", m_state.toString());
  }

  private void stateMachine() {
    Command currentCubeShooterCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentCubeShooterCommand = Idle();
          break;
        // Need to make DOWN_SETPOINT
        case CUBE_DOWN:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.DOWN_SETPOINT, -0.70d,
              -0.70d);
          break;
        case CUBE_UP:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.UP_SETPOINT, 0d, 0d);
          break;
        case CUBE_MID:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.CUBE_MID, 0d,
              0d);
          break;
        case CUBE_HIGH:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.UP_SETPOINT, 0.25d,
              0.25d);
          break;
        case CUBE_SPIT:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.CUBE_MID, .4, .4);
          break;
        case CUBE_SPIT_HIGH:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.UP_SETPOINT, .5, .5);
          break;
        case CALIBRATE:
          currentCubeShooterCommand = Calibrate();
          break;
        default:
          break;
      }
      m_lastState = m_state;

      if (currentCubeShooterCommand != null) {
        currentCubeShooterCommand.schedule();
      }

    }

  }

  public CubeShooterStates getState() {
    return m_state;
  }

  private Command Idle() {
    return Commands.runOnce(() -> {
      m_cubePivot.set(0d);
      m_topRoller.set(0d);
      m_bottomRoller.set(0d);
    }, this);
  }

  private Command Calibrate() {
    return Commands.run(() -> {
      m_cubePivot.setSmartCurrentLimit(5, 40);
      m_cubePivot.set(0.5);
    }, this).withTimeout(0.5)
        .andThen(() -> {
          m_cubePivot.getEncoder().setPosition(CubeShooterConstants.DEGREE_RANGE - 7.0);
          m_state = CubeShooterStates.CUBE_UP;
          m_cubePivot.setSmartCurrentLimit(40, 40);
        }, this);
  }

  private Command SetPivotPositionAndRollerSpeed(double position, double frontSpeed, double backSpeed) {
    m_cubePivotSetpoint = position; // variable exists for telemetry purposes
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_topRoller.set(frontSpeed);
          m_bottomRoller.set(backSpeed);
        }, this),
        Commands.run(() -> {
          double arbFF = MathUtil.clamp(calcFF(m_cubePivot.getEncoder().getPosition()), -1d, 1d);
          m_cubePivot.getPIDController().setReference(m_cubePivotSetpoint, ControlType.kPosition, 0, arbFF);
        }, this));
  }

  private double calcFF(double thetaDegrees) {
    double ff = CubeShooterConstants.kG * Math.cos(Math.toRadians(thetaDegrees));
    SmartDashboard.putNumber("Cube Shooter/Feed Forward", ff);
    return ff;
  }

  public Command setState(CubeShooterStates state) {
    return new InstantCommand(() -> m_state = state);
  }

  public static CubeShooter getInstance() {
    return m_instance = (m_instance == null) ? new CubeShooter() : m_instance;
  }
}
