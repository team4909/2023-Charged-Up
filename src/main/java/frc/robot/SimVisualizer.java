package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class SimVisualizer {

  private static SimVisualizer m_instance;

  private final double kWidth = 4d;
  private final double kHeight = 3d;
  private final double kLengthScaleFactor = 1.5;
  private final double kElevatorHeightOffsetInches = 7.65; // from cad

  private final Mechanism2d m_mechanismWindow;
  private final MechanismRoot2d m_elevatorRoot, m_cubeShooterRoot;
  private final MechanismLigament2d m_baseLigament, m_intakeLigament, m_cubeShooterLigament, m_wristLigament,
      m_elevatorLigament;

  public final Consumer<Double> intakeAngle;
  public final Consumer<Double> cubeShooterAngle;
  public final Consumer<Double> wristAngle;
  public final Consumer<Double> elevatorExtension;

  private SimVisualizer() {
    m_mechanismWindow = new Mechanism2d(kWidth, kHeight);
    m_mechanismWindow.setBackgroundColor(new Color8Bit(Color.kDimGray));
    m_baseLigament = m_mechanismWindow.getRoot("base_root", kWidth / 3d, kHeight / 6d)
        .append(new MechanismLigament2d("base", Units.inchesToMeters(26d) * kLengthScaleFactor, 0d));
    m_baseLigament.setColor(new Color8Bit(Color.kForestGreen));

    m_intakeLigament = m_baseLigament
        .append(new MechanismLigament2d("intake", IntakeConstants.SIM.ARM_LENGTH * kLengthScaleFactor, 90d, 15d,
            new Color8Bit(Color.kBlack)));
    m_cubeShooterRoot = m_mechanismWindow.getRoot("cube_shooter_root", kWidth / 3d, kHeight / 4d);
    m_cubeShooterLigament = m_cubeShooterRoot
        .append(new MechanismLigament2d("Cube Shooter", Units.inchesToMeters(26d) * kLengthScaleFactor, 110d));
    m_cubeShooterLigament.setColor(new Color8Bit(Color.kGray));

    // #region Elevator & End Effector
    m_elevatorRoot = m_mechanismWindow.getRoot("elevator_root", kWidth / 3d, kHeight / 6d);
    MechanismLigament2d elevatorBase = m_elevatorRoot
        .append(new MechanismLigament2d("elevator_base",
            Units.inchesToMeters(kElevatorHeightOffsetInches) * kLengthScaleFactor, 90d));
    elevatorBase.setColor(new Color8Bit(Color.kGhostWhite));
    MechanismLigament2d elevatorOuterStage = elevatorBase
        .append(new MechanismLigament2d("elevator_outer_stage",
            ElevatorConstants.SIM.RETRACTED_LENGTH * kLengthScaleFactor, -45d, 15d,
            new Color8Bit(Color.kForestGreen)));
    m_elevatorLigament = elevatorOuterStage
        .append(new MechanismLigament2d("elevator_inner_stage", 0d, 0d));
    m_elevatorLigament.setColor(new Color8Bit(Color.kBlack));

    m_wristLigament = m_elevatorLigament
        .append(new MechanismLigament2d("wrist", WristConstants.SIM.ARM_LENGTH * kLengthScaleFactor, -45d));
    m_wristLigament.setColor(new Color8Bit(Color.kBlack));
    // #endregion
    intakeAngle = inputDeg -> m_intakeLigament.setAngle(inputDeg);
    cubeShooterAngle = inputDeg -> m_cubeShooterLigament.setAngle(inputDeg);
    wristAngle = inputDeg -> m_wristLigament.setAngle(inputDeg - 45d);
    elevatorExtension = inputMeters -> m_elevatorLigament
        .setLength(inputMeters * kLengthScaleFactor);

    SmartDashboard.putData("Robot Mechanism Simulation", m_mechanismWindow);
  }

  public static SimVisualizer getInstance() {
    return m_instance = (m_instance == null) ? new SimVisualizer() : m_instance;
  }
}
