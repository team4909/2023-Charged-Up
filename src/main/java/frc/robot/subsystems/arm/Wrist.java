package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.SimVisualizer;

public class Wrist extends SubsystemBase {
  private static Wrist m_instance;
  private WristStates m_state, m_lastState;

  private final CANSparkMax m_wristMotor;
  private final SingleJointedArmSim m_wristSim;
  private final PIDController m_simWristPID;

  public enum WristStates {
    IDLE("Idle"),
    RETRACTED("Retracted"),
    HANDOFF_CONE("Handoff Cone"),
    HALF_DUNK("Half Dunk"),
    HALF_DUNK_HIGH("Half Dunk High"),
    DROPPING("Dropping"),
    SUBSTATION("Substation");

    String stateName;

    private WristStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return this.stateName;
    }
  }

  private Wrist() {
    m_state = WristStates.IDLE;
    m_wristMotor = new CANSparkMax(6, MotorType.kBrushless);
    configureHardware();

    if (Constants.SIM) {
      m_wristSim = new SingleJointedArmSim(
          DCMotor.getNEO(1),
          WristConstants.SIM.GEARING,
          WristConstants.SIM.MOI,
          WristConstants.SIM.ARM_LENGTH,
          Math.toRadians(-66d), // These angles are not exact, but add up to the degree range
          Math.toRadians(125d),
          true,
          VecBuilder.fill(Units.degreesToRadians(0.04)));
      m_simWristPID = new PIDController(0.003, 0.005d, 0d);
    } else {
      m_wristSim = null;
      m_simWristPID = null;
    }
  }

  public void periodic() {
    stateMachine();
    SmartDashboard.putString("Wrist/State", m_state.toString());
    SmartDashboard.putNumber("Wrist/Encoder Position (Degrees)", getWristEncoderPos());
    SmartDashboard.putNumber("Wrist/Motor Output", m_wristMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    double input = m_wristMotor.getAppliedOutput() * RobotController.getBatteryVoltage()
        + calcFF(m_wristMotor.getEncoder().getPosition());
    m_wristSim.setInput(input);
    m_wristSim.update(Constants.PERIODIC_LOOP_DURATION);
    double simAngleDegrees = Math.toDegrees(m_wristSim.getAngleRads());
    m_wristMotor.getEncoder().setPosition(simAngleDegrees);
    SmartDashboard.putNumber("Wrist/Sim Degrees", simAngleDegrees);
    SmartDashboard.putNumber("Wrist/Sim Input", input);
    SmartDashboard.putNumber("Wrist/Sim I", m_wristSim.getCurrentDrawAmps());
    SmartDashboard.putNumber("Wrist/Sim v", m_wristSim.getVelocityRadPerSec());
    SimVisualizer.getInstance().wristAngle.accept(simAngleDegrees);
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
          currentWristCommand = SetWristPosition(-55.175472); // -48.0
          break;
        case HALF_DUNK:
          currentWristCommand = SetWristPosition(40.0);
          break;
        case HALF_DUNK_HIGH:
          currentWristCommand = SetWristPosition(70.0);
          break;
        case DROPPING:
          currentWristCommand = SetWristPosition(0);
          break;
        case SUBSTATION:
          currentWristCommand = SetWristPosition(35);
        default:
          m_state = WristStates.IDLE;
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
    SmartDashboard.putNumber("Wrist/Setpoint", setpoint);
    return new RunCommand(() -> {
      if (Constants.SIM) {
        double voltage = m_simWristPID.calculate(m_wristMotor.getEncoder().getPosition(), setpoint);
        m_wristMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
      } else {
        setWristSetpoint(setpoint);
      }
    }, this);
  }

  private void setWristSetpoint(double setpoint) {
    m_wristMotor.getPIDController().setReference(setpoint, ControlType.kPosition, 0,
        calcFF(Units.degreesToRadians(m_wristMotor.getEncoder().getPosition())));
  }

  public Command setState(WristStates state) {
    return Commands.runOnce(() -> m_state = state);
  }

  private void configureHardware() {

    m_wristMotor.restoreFactoryDefaults();
    m_wristMotor.setSmartCurrentLimit(50, 60);
    m_wristMotor.setInverted(true);
    m_wristMotor.getPIDController().setP(WristConstants.kP);
    m_wristMotor.getPIDController().setD(WristConstants.kD);

    m_wristMotor.getPIDController().setOutputRange(-WristConstants.OUTPUT_LIMIT, WristConstants.OUTPUT_LIMIT);
    m_wristMotor.setIdleMode(IdleMode.kBrake);

    m_wristMotor.getPIDController().setSmartMotionAllowedClosedLoopError(200, 0);

    m_wristMotor.getEncoder().setPositionConversionFactor(WristConstants.DEGREES_PER_TICK);
    m_wristMotor.getEncoder().setPosition(121);

  }

  private double getWristEncoderPos() {
    return m_wristMotor.getEncoder().getPosition();
  }

  private double calcFF(double thetaDegrees) {
    double ff = WristConstants.kG * Math.cos(Math.toRadians(thetaDegrees));
    SmartDashboard.putNumber("Wrist/Feed Forward", ff);
    return MathUtil.clamp(ff, -1d, 1d);
  }

  public static Wrist getInstance() {
    if (m_instance == null) {
      m_instance = new Wrist();
    }
    return m_instance;

  }

}