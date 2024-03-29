package frc.robot.subsystems.elevator;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bioniclib.CANConfigurator;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.SimVisualizer;

public class Elevator extends SubsystemBase {

  private static Elevator m_instance = null;
  private ElevatorStates m_state, m_lastState;
  private final WPI_TalonFX m_leftExtensionMotor;
  private final WPI_TalonFX m_rightExtensionMotor;
  private final TalonFXSimCollection m_extensionMotorSim;
  private final ElevatorSim m_elevatorSim;
  private final ElevatorFeedforward m_elevatorFF;

  private double m_feedForwardVolts = 0d;

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
    m_leftExtensionMotor = new WPI_TalonFX(ElevatorConstants.LEFT_MOTOR, Constants.CANFD_BUS);
    m_rightExtensionMotor = new WPI_TalonFX(ElevatorConstants.RIGHT_MOTOR, Constants.CANFD_BUS);
    configHardware();
    m_elevatorFF = new ElevatorFeedforward(
        ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    m_state = ElevatorStates.IDLE;

    if (Constants.SIM) {
      m_extensionMotorSim = m_leftExtensionMotor.getSimCollection();
      m_elevatorSim = new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorConstants.SIM.GEARING,
          ElevatorConstants.SIM.CARRIAGE_MASS,
          ElevatorConstants.SIM.DRUM_RADIUS,
          0,
          ElevatorConstants.METER_RANGE,
          false);
    } else {
      m_extensionMotorSim = null;
      m_elevatorSim = null;
    }
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putNumber("Elevator/Closed Loop Error",
        m_leftExtensionMotor.getClosedLoopError() * ElevatorConstants.METERS_PER_TICK);
    SmartDashboard.putNumber("Elevator/Velocity",
        m_leftExtensionMotor.getSelectedSensorVelocity() * ElevatorConstants.METERS_PER_TICK);
    SmartDashboard.putNumber("Elevator/Position Meters",
        m_leftExtensionMotor.getSelectedSensorPosition() * ElevatorConstants.METERS_PER_TICK);
    SmartDashboard.putString("Elevator/State", m_state.toString());
  }

  @Override
  public void simulationPeriodic() {
    m_extensionMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    double input = m_extensionMotorSim.getMotorOutputLeadVoltage();
    m_elevatorSim.setInput(input);
    m_elevatorSim.update(Constants.PERIODIC_LOOP_DURATION);
    int sensorPosition = (int) (m_elevatorSim.getPositionMeters() / ElevatorConstants.METERS_PER_TICK);
    m_extensionMotorSim.setIntegratedSensorRawPosition(sensorPosition);
    int sensorVelocity = (int) (m_elevatorSim.getVelocityMetersPerSecond() / 10 / ElevatorConstants.METERS_PER_TICK);
    m_extensionMotorSim.setIntegratedSensorVelocity(sensorVelocity);

    SmartDashboard.putNumber("Elevator/Sim Meters", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Elevator/Sim Input", input);
    SmartDashboard.putNumber("Elevator/Sim I", m_elevatorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Elevator/Sim v", m_elevatorSim.getVelocityMetersPerSecond());
    SimVisualizer.getInstance().elevatorExtension.accept(m_elevatorSim.getPositionMeters());
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
          currentElevatorCommand = SetSetpoint(ElevatorConstants.DOUBLE_SUBSTATION_SETPOINT);
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
    SmartDashboard.putNumber("Elevator/Position Setpoint", setpointMeters);
    return new RunCommand(() -> {
      m_leftExtensionMotor.set(TalonFXControlMode.MotionMagic, setpointMeters / ElevatorConstants.METERS_PER_TICK,
          DemandType.ArbitraryFeedForward, m_feedForwardVolts / Constants.NOMINAL_VOLTAGE); // Converting ff to percent
      double elevatordesiredVelMPS = m_leftExtensionMotor.getActiveTrajectoryVelocity() * 10
          * ElevatorConstants.METERS_PER_TICK;
      m_feedForwardVolts = m_elevatorFF.calculate(elevatordesiredVelMPS);
      SmartDashboard.putNumber("Elevator/Motion Magic Velocity", elevatordesiredVelMPS);
      SmartDashboard.putNumber("Elevator/Feed Forward V", m_feedForwardVolts);
    }, this);
  }

  private void configHardware() {
    CANConfigurator<ErrorCode> talonManager = new CANConfigurator<>("Elevator", ErrorCode.class);
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor.configFactoryDefault());
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor.setSelectedSensorPosition(0));
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor.config_kP(0, ElevatorConstants.kP));
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor.config_kD(0, ElevatorConstants.kD));
    talonManager.actionConsumer
        .accept(() -> m_leftExtensionMotor.configClosedLoopPeakOutput(0, ElevatorConstants.OUTPUT_LIMIT));
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor
        .configMotionCruiseVelocity(ElevatorConstants.MOTION_CRUISE_VELOCITY / ElevatorConstants.METERS_PER_TICK / 10));
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor
        .configMotionAcceleration(ElevatorConstants.MOTION_ACCELERATION / ElevatorConstants.METERS_PER_TICK / 10));
    talonManager.actionConsumer.accept(() -> m_leftExtensionMotor.configMotionSCurveStrength(7));
    talonManager.actionConsumer
        .accept(() -> m_leftExtensionMotor.configAllowableClosedloopError(0, ElevatorConstants.CLOSED_LOOP_TOLERANCE));
    m_leftExtensionMotor.set(TalonFXControlMode.MotionMagic, 0);
    m_leftExtensionMotor.setInverted(false);
    // This stops stale frames in sim
    if (Constants.SIM) {
      m_leftExtensionMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10);
      m_leftExtensionMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 50);
    }
    talonManager.actionConsumer.accept(() -> m_rightExtensionMotor.configFactoryDefault());
    m_rightExtensionMotor.setInverted(TalonFXInvertType.OpposeMaster);
    m_rightExtensionMotor.follow(m_leftExtensionMotor);
    talonManager.forceConfig();
  }

  public Command setState(ElevatorStates state) {
    return Commands.runOnce(() -> m_state = state);
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
