package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bioniclib.SparkManager;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private static Claw m_instance;
  private ClawStates m_state, m_lastState;
  private final CANSparkMax m_clawMotor;
  private final SparkMaxAbsoluteEncoder m_clawEncoder;

  public enum ClawStates {
    IDLE("Idle"),
    CLOSED("Closed"),
    OPEN("Open"),
    HANDOFF("Handoff"),
    SCORE("Score");

    String stateName;

    private ClawStates(String name) {
      this.stateName = name;
    }

    public String toString() {
      return this.stateName;
    }
  }

  private Claw() {
    m_state = ClawStates.IDLE;
    m_clawMotor = new CANSparkMax(5, MotorType.kBrushless);
    m_clawEncoder = m_clawMotor.getAbsoluteEncoder(Type.kDutyCycle);

    SparkManager sparkManager = new SparkManager("Clamp with Absolute Encoder");
    Runnable config = () -> {
      sparkManager.statusTracker.accept(m_clawMotor.restoreFactoryDefaults());
      sparkManager.statusTracker.accept(m_clawMotor.getPIDController().setFeedbackDevice(m_clawEncoder));
      sparkManager.statusTracker.accept(m_clawMotor.getPIDController().setP(ClawConstants.kP));
      sparkManager.statusTracker.accept(
          m_clawMotor.getPIDController().setOutputRange(-ClawConstants.OUTPUT_LIMIT,
              ClawConstants.OUTPUT_LIMIT));
      sparkManager.statusTracker.accept(m_clawMotor.setIdleMode(IdleMode.kCoast));
      sparkManager.statusTracker.accept(m_clawMotor.setSmartCurrentLimit(10));
      m_clawMotor.setInverted(false);

      sparkManager.statusTracker.accept(m_clawEncoder.setZeroOffset(0.4));
    };

    sparkManager.setConfigRunnable(config);
    sparkManager.forceConfig();
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putString("Claw/State", m_state.toString());
    SmartDashboard.putNumber("Claw/Encoder Position", m_clawEncoder.getPosition());
  }

  private void stateMachine() {
    Command currentClawCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentClawCommand = Idle();
          break;
        case CLOSED:
          currentClawCommand = SetClawPos(0);
          break;
        case OPEN:
          currentClawCommand = SetClawPos(0.15);
          break;
        case HANDOFF:
          currentClawCommand = SetClawPos(0.2);
          break;
        case SCORE:
          currentClawCommand = SetClawPos(0.1);
          break;
        default:
          m_state = ClawStates.IDLE;
          break;
      }
    }

    m_lastState = m_state;

    if (currentClawCommand != null) {
      currentClawCommand.schedule();
    }
  }

  private Command Idle() {
    return Commands.runOnce(() -> m_clawMotor.set(0.0), this);
  }

  private Command SetClawPos(double setpoint) {
    return new InstantCommand(() -> {
      SmartDashboard.putNumber("Claw/Setpoint", setpoint);
      m_clawMotor.getPIDController().setReference(setpoint, ControlType.kPosition);
    }, this);
  }

  public Command setState(ClawStates state) {
    return Commands.runOnce(() -> m_state = state);
  }

  public static Claw getInstance() {
    if (m_instance == null) {
      m_instance = new Claw();
    }
    return m_instance;

  }
}
