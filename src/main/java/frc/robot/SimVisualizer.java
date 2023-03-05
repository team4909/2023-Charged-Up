package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class SimVisualizer {

  private static SimVisualizer m_instance;

  private final double kWidth = 4d;
  private final double kHeight = 3d;

  private final Mechanism2d m_mechanismWindow;
  private final MechanismRoot2d m_intakeRoot, m_wristRoot;
  private final MechanismLigament2d m_intakeLigament, m_wristLigament;

  public final Consumer<Double> intakeAngle;
  public final Consumer<Double> wristAngle;

  private SimVisualizer() {
    m_mechanismWindow = new Mechanism2d(kWidth, kHeight);
    m_intakeRoot = m_mechanismWindow.getRoot("intake_root", kWidth / 2, kHeight / 2);
    m_intakeLigament = m_intakeRoot.append(new MechanismLigament2d("intake", IntakeConstants.Sim.ARM_LENGTH, 90));
    m_wristRoot = m_mechanismWindow.getRoot("wrist_root", kWidth / 2, kHeight / 1.5);
    m_wristLigament = m_wristRoot.append(new MechanismLigament2d("wrist", 1d, 0));
    SmartDashboard.putData("Robot Mechanism Simulation", m_mechanismWindow);

    intakeAngle = inputDeg -> m_intakeLigament.setAngle(inputDeg);
    wristAngle = inputDeg -> m_wristLigament.setAngle(inputDeg);
  }

  public static SimVisualizer getInstance() {
    return m_instance = (m_instance == null) ? new SimVisualizer() : m_instance;
  }
}
