package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private static Elevator m_instance = null;
  private ElevatorStates m_state, m_lastState;
  private final TalonFX m_leftExtensionMotor;
  private final TalonFX m_rightExtensionMotor;

  private static final ElevatorFeedforward m_elevatorFF = new ElevatorFeedforward(
      0.40921, 0.12654, 1.3374, 0.031771);

  // final double sensorUnitsToMetersSecondsCoeff = (1d / 2048d) * (0.0889 / 1d) *
  // (100d / 0.1) * (60d / 1d);

  public enum ElevatorStates {
    IDLE("Idle"),
    TOP("Top Node Extension"),
    MID_CUBE("Mid Cube"),
    MID_CONE("Mid Cone"),
    RETRACT("Retracted"),
    SUBSTATION("Substation"),
    DOUBLE_SUBSTATION("Double Substation");

    String stateName;

    private ElevatorStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return this.stateName;
    }
  }

  private Elevator() {
    m_leftExtensionMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR, Constants.CANFD_BUS);
    m_rightExtensionMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR, Constants.CANFD_BUS);
    configHardware();
    m_rightExtensionMotor.setInverted(true);
    m_state = ElevatorStates.IDLE;
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putNumber("Elevator Closed Loop Error", m_leftExtensionMotor.getClosedLoopError());
    SmartDashboard.putNumber("Elevator Position Meters",
        m_leftExtensionMotor.getSelectedSensorPosition() * ElevatorConstants.METERS_PER_TICK);
    SmartDashboard.putString("Elevator State", m_state.toString());
  }

  private void stateMachine() {
    Command currentElevatorCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentElevatorCommand = Idle();
          break;
        case TOP:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.TOP_SETPOINT);
          break;
        case MID_CUBE:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.MID_CUBE_SETPOINT);
          break;
        case MID_CONE:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.MID_CONE_SETPOINT);
          break;
        case RETRACT:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.BOTTOM_SETPOINT);
          break;
        case SUBSTATION:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.SUBSTATION_SETPOINT);
          break;
        case DOUBLE_SUBSTATION:
          currentElevatorCommand = SetSetpoint(ElevatorConstants.SUBSTATION_SETPOINT);
        default:
          m_state = ElevatorStates.IDLE;
          break;
      }
    }

    m_lastState = m_state;

    if (currentElevatorCommand != null) {
      currentElevatorCommand.schedule();
    }
  }

  private Command Idle() {
    return new InstantCommand(() -> m_leftExtensionMotor.set(TalonFXControlMode.PercentOutput, 0), this);
  }

  private Command SetSetpoint(double setpointMeters) {
    double ff = m_elevatorFF
        .calculate(m_leftExtensionMotor.getSelectedSensorVelocity() * 10 * ElevatorConstants.METERS_PER_TICK);
    SmartDashboard.putNumber("Elevator Setpoint", setpointMeters);
    return new RunCommand(() -> {
      SmartDashboard.putNumber("Elevator FF", ff);
      m_leftExtensionMotor.set(TalonFXControlMode.Position, setpointMeters / ElevatorConstants.METERS_PER_TICK);
    }, this);
  }

  private void configHardware() {
    m_leftExtensionMotor.configFactoryDefault();
    m_leftExtensionMotor.setSelectedSensorPosition(0);
    m_leftExtensionMotor.config_kP(0, ElevatorConstants.kP);
    m_leftExtensionMotor.config_kD(0, ElevatorConstants.kD);
    m_leftExtensionMotor.configClosedLoopPeakOutput(0, ElevatorConstants.OUTPUT_LIMIT);
    m_leftExtensionMotor.setInverted(false);

    m_rightExtensionMotor.configFactoryDefault();
    m_rightExtensionMotor.setInverted(TalonFXInvertType.OpposeMaster);
    m_rightExtensionMotor.follow(m_leftExtensionMotor);
  }

  public void setState(ElevatorStates state) {
    m_state = state;
  }

  public Command setState2(ElevatorStates state) {
    return new InstantCommand(() -> m_state = state);
  }

  public ElevatorStates getState() {
    return m_state;
  }

  public static Elevator getInstance() {
    if (m_instance == null) {
      m_instance = new Elevator();
    }
    return m_instance;
  }
}
