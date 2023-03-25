package frc.robot.subsystems.drivetrain.auto;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.WristStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.CubeShooter;
import frc.robot.subsystems.intake.CubeShooter.CubeShooterStates;
import frc.robot.subsystems.intake.CubeShooter.ShooterLevels;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;

public class AutoRoutines {

  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Wrist m_wrist = Wrist.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Claw m_claw = Claw.getInstance();
  private final CubeShooter m_cubeShooter = CubeShooter.getInstance();

  public final Auto TEST = new Auto(
      loadTrajectory(new DriveTrajectory("Test2", true)));
  public final Auto BLANK_AUTO = new Auto();
  public final Auto SCORE_CONE_CHARGE_STATION_COMMUNITY = new Auto(
      SCORE_CONE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("ChargeStationStraight", true, 1.5)),
      loadTrajectory(new DriveTrajectory("BackChargeStation", false, 1.5)),
      m_drivetrain.setState(DrivetrainStates.AUTO_BALANCE));
  public final Auto ONE_PIECE_CHARGE_STATION = new Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopNodeToTopPiece", true)),
          Commands.sequence(
              Commands.waitSeconds(3),
              INTAKE_CONE())),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopPieceToTopNode", false)),
          HANDOFF()),
      SCORE_CONE(ElevatorStates.TOP));
  public final Auto ONE_CUBE = new Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopNodeToTopCube", true)),
          INTAKE_CUBE()),
      loadTrajectory(new DriveTrajectory("TopCubeToTopCubeNode", false)),
      SCORE_CUBE_HIGH());
  // Commands.parallel(
  // loadTrajectory(new DriveTrajectory("TopCubeNodeToSecondPiece", false)),
  // INTAKE_CUBE().beforeStarting(Commands.waitSeconds(3))));

  private Command loadTrajectory(DriveTrajectory traj) {
    return Commands.waitUntil(m_drivetrain.isTrajectoryFinished)
        .deadlineWith(m_drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
            new HashMap<>(
                Map.of("Trajectory", traj.trajectory(), "IsFirstPath", traj.isFirstPath()))));
  }

  private class Auto extends SequentialCommandGroup {
    public Auto(Command... events) {
      super.addCommands(events);
    }
  }

  private final Command INIT() {
    return Commands.sequence(
        m_claw.setState(ClawStates.CLOSED),
        m_intake.setState(IntakeStates.CALIBRATE),
        m_cubeShooter.setState(CubeShooterStates.CALIBRATE),
        Commands.waitSeconds(1),
        m_intake.setState(IntakeStates.RETRACTED),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  private final Command SCORE_CONE(ElevatorStates extensionLevel) {
    return Commands.sequence(
        m_elevator.setState(extensionLevel),
        Commands.waitSeconds(1.5),
        m_wrist.setState(WristStates.DROPPING),
        Commands.waitSeconds(0.75),
        m_claw.setState(ClawStates.OPEN),
        Commands.waitSeconds(0.2),
        m_wrist.setState(WristStates.RETRACTED),
        m_elevator.setState(ElevatorStates.RETRACT))
        .alongWith(INIT());
  }

  private final Command INTAKE_CONE() {
    return Commands.sequence(
        m_intake.setState(IntakeStates.INTAKE_CONE),
        Commands.waitSeconds(2d),
        m_intake.setState(IntakeStates.HANDOFF));
  }

  private final Command INTAKE_CUBE() {
    return Commands.sequence(
        m_cubeShooter.setState(CubeShooterStates.INTAKE),
        Commands.waitSeconds(4),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  private final Command SCORE_CUBE_HIGH() {
    return Commands.sequence(
        m_cubeShooter.Config(ShooterLevels.HIGH),
        m_cubeShooter.setState(CubeShooterStates.SCORE),
        Commands.waitSeconds(0.5),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  public final Command HANDOFF() {
    return Commands.sequence(
        m_claw.setState(ClawStates.OPEN),
        m_wrist.setState(WristStates.HANDOFF_CONE),
        Commands.waitSeconds(1),
        m_claw.setState(ClawStates.CLOSED),
        Commands.waitSeconds(0.5),
        m_intake.setState(IntakeStates.SPIT_CONE),
        Commands.waitSeconds(0.1),
        m_wrist.setState(WristStates.RETRACTED),
        m_intake.setState(IntakeStates.RETRACTED));
  }
}
