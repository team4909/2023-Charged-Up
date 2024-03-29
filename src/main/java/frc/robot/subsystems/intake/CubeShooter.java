package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bioniclib.CANConfigurator;
import frc.robot.Constants.CubeShooterConstants;

public class CubeShooter extends SubsystemBase {
  private static CubeShooter m_instance;
  private CubeShooterStates m_state, m_lastState;
  private final CANSparkMax m_cubePivot, m_topRoller, m_bottomRoller;
  private double m_pivotSetpoint, m_frontRollerSetpoint, m_backRollerSetpoint;

  public DoubleSupplier topRollerOutputCurrent;
  public boolean isIntaking = false;

  public enum ShooterLevels {
    LOW(CubeShooterConstants.RETRACTED_SETPOINT, 0.2, 0.2),
    MID(CubeShooterConstants.CUBE_MID, 0.45, 0.45),
    HIGH(CubeShooterConstants.RETRACTED_SETPOINT, 0.75, 0.75);

    double pivotSetpoint, frontRollerSetpoint, backRollerSetpoint;

    private ShooterLevels(double pivot, double front, double back) {
      pivotSetpoint = pivot;
      frontRollerSetpoint = front;
      backRollerSetpoint = back;
    }
  }

  public enum CubeShooterStates {
    IDLE("Idle"),
    INTAKE("Intake"),
    RETRACTED("Retracted"),
    SPIT("Spit"),
    SCORE("Score"),
    CALIBRATE("Calibrate");

    String stateName;

    private CubeShooterStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return stateName;
    }
  }

  private CubeShooter() {
    m_cubePivot = new CANSparkMax(CubeShooterConstants.PIVOT_MOTOR, MotorType.kBrushless);
    m_topRoller = new CANSparkMax(CubeShooterConstants.TOP_ROLLER_MOTOR, MotorType.kBrushless);
    m_bottomRoller = new CANSparkMax(CubeShooterConstants.BOTTOM_ROLLER_MOTOR, MotorType.kBrushless);

    CANConfigurator<REVLibError> sparkManager = new CANConfigurator<>("Cube Shooter Pivot & Rollers",
        REVLibError.class);
    sparkManager.actionConsumer.accept(() -> m_cubePivot.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_topRoller.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_bottomRoller.restoreFactoryDefaults());

    sparkManager.actionConsumer.accept(() -> m_cubePivot.getPIDController().setP(CubeShooterConstants.kP));
    sparkManager.actionConsumer.accept(() -> m_cubePivot.getPIDController().setD(CubeShooterConstants.kD));
    sparkManager.actionConsumer
        .accept(() -> m_cubePivot.getPIDController().setOutputRange(-CubeShooterConstants.OUTPUT_LIMIT,
            CubeShooterConstants.OUTPUT_LIMIT));

    sparkManager.actionConsumer.accept(() -> m_cubePivot.setSmartCurrentLimit(40, 40));
    sparkManager.actionConsumer
        .accept(() -> m_cubePivot.getEncoder().setPositionConversionFactor(CubeShooterConstants.DEGREES_PER_TICK));
    sparkManager.actionConsumer
        .accept(() -> m_cubePivot.getEncoder().setPosition(CubeShooterConstants.DEGREE_RANGE - 7.0));
    m_cubePivot.setInverted(false);
    sparkManager.forceConfig();
    m_state = CubeShooterStates.IDLE;
    topRollerOutputCurrent = () -> m_topRoller.getOutputCurrent();
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putNumber("Cube Shooter/Pivot Error",
        m_cubePivot.getEncoder().getPosition() - m_pivotSetpoint);
    SmartDashboard.putNumber("Cube Shooter/Pivot Setpoint", m_pivotSetpoint);
    SmartDashboard.putNumber("Cube Shooter/Pivot Encoder Position", m_cubePivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Cube Shooter/Pivot Output", m_cubePivot.getAppliedOutput());
    SmartDashboard.putNumber("Cube Shooter/Pivot Current", m_cubePivot.getOutputCurrent());
    SmartDashboard.putNumber("Cube Shooter/Top Roller Current", m_topRoller.getOutputCurrent());
    SmartDashboard.putString("Cube Shooter/State", m_state.toString());
  }

  private void stateMachine() {
    Command currentCubeShooterCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentCubeShooterCommand = Idle();
          break;
        case INTAKE:
          isIntaking = true;
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.DOWN_SETPOINT, -0.7, -0.7,
              true).finallyDo((i) -> isIntaking = false);
          break;
        case RETRACTED:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.RETRACTED_SETPOINT, 0.0, 0.0,
              true);
          break;
        case SCORE:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(m_pivotSetpoint, m_frontRollerSetpoint,
              m_backRollerSetpoint, true);
          break;
        case SPIT:
          currentCubeShooterCommand = SetPivotPositionAndRollerSpeed(CubeShooterConstants.DOWN_SETPOINT, 0.2, 0.2,
              false);
          break;
        default:
          currentCubeShooterCommand = Idle();
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

  private Command SetPivotPositionAndRollerSpeed(double position, double frontSpeed, double backSpeed,
      boolean spinFirst) {
    m_pivotSetpoint = position; // variable exists for telemetry purposes
    Command rollers = RunRollers(frontSpeed, backSpeed);
    Command pivot = Pivot(position);
    return spinFirst ? Commands.sequence(rollers, pivot)
        : Commands.sequence(pivot.andThen(Commands.waitSeconds(0.5)), rollers);
  }

  public Command Configure(ShooterLevels shooterLevel) {
    return Commands.runOnce(() -> {
      m_pivotSetpoint = shooterLevel.pivotSetpoint;
      m_frontRollerSetpoint = shooterLevel.frontRollerSetpoint;
      m_backRollerSetpoint = shooterLevel.backRollerSetpoint;
    }).andThen(
        () -> Pivot(m_pivotSetpoint).schedule());
  };

  Command RunRollers(double frontSpeed, double backSpeed) {
    return Commands.runOnce(() -> {
      m_topRoller.set(frontSpeed);
      m_bottomRoller.set(backSpeed);
    }, this);
  }

  Command Pivot(double position) {
    return Commands.run(() -> {
      double arbFF = MathUtil.clamp(calcFF(m_cubePivot.getEncoder().getPosition()), -1d, 1d);
      m_cubePivot.getPIDController().setReference(m_pivotSetpoint, ControlType.kPosition, 0, arbFF);
    }, this).until(() -> (position - m_cubePivot.getEncoder().getPosition() < 1.5));
  }

  private double calcFF(double thetaDegrees) {
    double ff = CubeShooterConstants.kG * Math.cos(Math.toRadians(thetaDegrees));
    SmartDashboard.putNumber("Cube Shooter/Feed Forward", ff);
    return ff;
  }

  public Command setState(CubeShooterStates state) {
    return Commands.runOnce(() -> m_state = state);
  }

  public static CubeShooter getInstance() {
    return m_instance = (m_instance == null) ? new CubeShooter() : m_instance;
  }
}
