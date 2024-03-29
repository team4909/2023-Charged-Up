package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bioniclib.CANConfigurator;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.SimVisualizer;

public class Intake extends SubsystemBase {

  private static Intake m_instance;
  private IntakeStates m_state, m_lastState;
  private final CANSparkMax m_pivot, m_frontRoller, m_backRoller;
  private double m_pivotSetpoint;
  private final SingleJointedArmSim m_pivotSim;
  private final PIDController m_simPivotPID;

  public DoubleSupplier backRollerOutputCurrent;
  public boolean isIntaking;

  public enum IntakeStates {
    IDLE("Idle"),
    RETRACTED("Retracted"),
    INTAKE("Intake"),
    SPIT("Spit"),
    HANDOFF("Handoff"),
    HOLDING("Holding");

    String stateName;

    private IntakeStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return stateName;
    }
  }

  private Intake() {
    m_pivot = new CANSparkMax(IntakeConstants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
    m_frontRoller = new CANSparkMax(IntakeConstants.FRONT_ROLLER_MOTOR, MotorType.kBrushless);
    m_backRoller = new CANSparkMax(IntakeConstants.BACK_ROLLER_MOTOR, MotorType.kBrushless);

    CANConfigurator<REVLibError> sparkManager = new CANConfigurator<>("Intake Pivot & Rollers",
        REVLibError.class);
    sparkManager.actionConsumer.accept(() -> m_pivot.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_pivot.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_frontRoller.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_backRoller.restoreFactoryDefaults());

    sparkManager.actionConsumer.accept(() -> m_pivot.getPIDController().setP(IntakeConstants.kP));
    sparkManager.actionConsumer
        .accept(() -> m_pivot.getPIDController().setOutputRange(-IntakeConstants.OUTPUT_LIMIT,
            IntakeConstants.OUTPUT_LIMIT));

    sparkManager.actionConsumer.accept(() -> m_pivot.setSmartCurrentLimit(40, 40));
    sparkManager.actionConsumer
        .accept(() -> m_pivot.getEncoder().setPositionConversionFactor(IntakeConstants.DEGREES_PER_TICK));
    sparkManager.actionConsumer.accept(() -> m_pivot.getEncoder().setPosition(110d));
    m_pivot.setInverted(false);

    sparkManager.forceConfig();

    m_state = IntakeStates.IDLE;
    backRollerOutputCurrent = () -> m_backRoller.getOutputCurrent();

    if (Constants.SIM) {
      m_pivotSim = new SingleJointedArmSim(
          DCMotor.getNEO(2),
          IntakeConstants.SIM.GEARING,
          IntakeConstants.SIM.MOI / 5,
          IntakeConstants.SIM.ARM_LENGTH,
          Math.toRadians(0),
          Math.toRadians(IntakeConstants.DEGREE_RANGE),
          true,
          VecBuilder.fill(Units.degreesToRadians(0.04)));
      m_simPivotPID = new PIDController(0.0045, 0.005d, 0d);
    } else {
      m_pivotSim = null;
      m_simPivotPID = null;
    }
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putString("Intake/Intake State", m_state.toString());
    SmartDashboard.putNumber("Intake/Pivot Error", m_pivot.getEncoder().getPosition() - m_pivotSetpoint);
    SmartDashboard.putNumber("Intake/Pivot Setpoint", m_pivotSetpoint);
    SmartDashboard.putNumber("Intake/Pivot Encoder Position", m_pivot.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake/Pivot Output", m_pivot.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Pivot Current", m_pivot.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Front Roller Current", m_frontRoller.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Back Roller Current", m_backRoller.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    double input = m_pivot.getAppliedOutput() * RobotController.getBatteryVoltage()
        + calcFF(m_pivot.getEncoder().getPosition());
    m_pivotSim.setInput(input);
    m_pivotSim.update(Constants.PERIODIC_LOOP_DURATION);
    double simAngleDegrees = Math.toDegrees(m_pivotSim.getAngleRads());
    m_pivot.getEncoder().setPosition(simAngleDegrees);
    SmartDashboard.putNumber("Intake/Sim Degrees", simAngleDegrees);
    SmartDashboard.putNumber("Intake/Sim Input", input);
    SmartDashboard.putNumber("Intake/Sim I", m_pivotSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Intake/Sim v", m_pivotSim.getVelocityRadPerSec());
    SimVisualizer.getInstance().intakeAngle.accept(simAngleDegrees);
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
        case INTAKE:
          isIntaking = true;
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.CONE_SETPOINT, 0.75, 0.75)
              .finallyDo((i) -> isIntaking = false);
          break;
        case SPIT:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.SPIT_CONE_SETPOINT, 0.3d, -0.3d);
          break;
        case HANDOFF:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.HANDOFF_SETPOINT, 0.2, 0.15);
          break;
        case HOLDING:
          currentIntakeCommand = SetPivotPositionAndRollerSpeed(IntakeConstants.HANDOFF_SETPOINT, 0.2, 0.15);
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
      m_pivot.set(0d);
      m_frontRoller.set(0d);
      m_backRoller.set(0d);
    }, this);
  }

  private Command SetPivotPositionAndRollerSpeed(double position, double frontSpeed, double backSpeed) {
    m_pivotSetpoint = position; // variable exists for telemetry purposes
    return Commands.sequence(
        Commands.runOnce(() -> {
          m_frontRoller.set(frontSpeed);
          m_backRoller.set(backSpeed);
        }, this),
        Commands.run(() -> {
          double arbFF = MathUtil.clamp(calcFF(m_pivot.getEncoder().getPosition()), -1d, 1d);
          if (Constants.SIM) {
            // Rev sim does not support position control
            double voltage = m_simPivotPID.calculate(m_pivot.getEncoder().getPosition(), m_pivotSetpoint);
            m_pivot.getPIDController().setReference(voltage, ControlType.kVoltage);
          } else {
            m_pivot.getPIDController().setReference(m_pivotSetpoint, ControlType.kPosition, 0, arbFF);
          }
        }, this));
  }
  // #endregion

  private double calcFF(double thetaDegrees) {
    double ff = IntakeConstants.kG * Math.cos(Math.toRadians(thetaDegrees));
    SmartDashboard.putNumber("Intake/Feed Forward", ff);
    return ff;
  }

  public Command setState(IntakeStates state) {
    return Commands.runOnce(() -> m_state = state);
  }

  public IntakeStates getState() {
    return m_state;
  }

  public static Intake getInstance() {
    return m_instance = (m_instance == null) ? new Intake() : m_instance;
  }
}
