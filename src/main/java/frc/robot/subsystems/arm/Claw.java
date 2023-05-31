package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private static Claw m_instance;
  private ClawStates m_state, m_lastState;
  private final CANSparkMax m_clawMotor;

  public enum ClawStates {
    IDLE("Idle"),
    INTAKING("Intaking"),
    SPITTING("Spitting");

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

    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.getPIDController().setOutputRange(-ClawConstants.OUTPUT_LIMIT, ClawConstants.OUTPUT_LIMIT);
    m_clawMotor.getPIDController().setP(1);
    m_clawMotor.setIdleMode(IdleMode.kCoast);
    m_clawMotor.setInverted(false);
    m_clawMotor.setSmartCurrentLimit(10);
  }

  @Override
  public void periodic() {
    stateMachine();
    SmartDashboard.putString("Claw/State", m_state.toString());
    SmartDashboard.putNumber("Claw/Position", m_clawMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Claw/Velocity", m_clawMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Claw/Current", m_clawMotor.getOutputCurrent());
  }

  private void stateMachine() {
    Command currentClawCommand = null;
    if (!m_state.equals(m_lastState)) {
      switch (m_state) {
        case IDLE:
          currentClawCommand = Idle();
          break;
        case INTAKING:
          currentClawCommand = SetSpeed(1);
          break;
        case SPITTING:
          currentClawCommand = SetSpeed(-0.5);
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

  private Command SetSpeed(double speed) {
    return this.runOnce(() -> {
      SmartDashboard.putNumber("Claw/Setpoint Velocity", speed);
      m_clawMotor.set(speed);
      // m_clawMotor.getPIDController().setReference(5, ControlType.kVoltage);
    });
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
