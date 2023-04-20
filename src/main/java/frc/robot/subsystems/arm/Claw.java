package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.bioniclib.CANConfigurator;
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
    CANConfigurator<REVLibError> sparkManager = new CANConfigurator<>("Claw", REVLibError.class);
    sparkManager.actionConsumer.accept(() -> m_clawMotor.restoreFactoryDefaults());
    sparkManager.actionConsumer.accept(() -> m_clawMotor.getPIDController().setFeedbackDevice(m_clawEncoder));
    sparkManager.actionConsumer.accept(() -> m_clawMotor.getPIDController().setP(ClawConstants.kP));
    sparkManager.actionConsumer.accept(
        () -> m_clawMotor.getPIDController().setOutputRange(-ClawConstants.OUTPUT_LIMIT, ClawConstants.OUTPUT_LIMIT));
    sparkManager.actionConsumer.accept(() -> m_clawMotor.setIdleMode(IdleMode.kCoast));
    sparkManager.actionConsumer.accept(() -> m_clawMotor.setSmartCurrentLimit(15));
    sparkManager.actionConsumer.accept(() -> m_clawEncoder.setZeroOffset(0.4));
    m_clawMotor.setInverted(false);
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
          currentClawCommand = SetClawPos(0.13);
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
