package frc.robot.subsystems.drivetrain.auto;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.subsystems.leds.LEDs;

public class AutoRoutines {

  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Wrist m_wrist = Wrist.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Claw m_claw = Claw.getInstance();
  private final CubeShooter m_cubeShooter = CubeShooter.getInstance();
  private final LEDs m_leds = LEDs.getInstance();
  public final Command TEST = Auto(
      loadTrajectory(new DriveTrajectory("Test2", true)));

  public final Command BLANK_AUTO = Auto();

  public final Command ONE_CONE_CHARGE_STATION = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("ChargeStationStraight", true, 1.5)),
      loadTrajectory(new DriveTrajectory("BackChargeStation", false, 1.5)),
      m_drivetrain.setState(DrivetrainStates.AUTO_BALANCE));

  public final Command ONE_CUBE_CHARGE_STATION = Auto(
      SCORE_CUBE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("ChargeStationStraight", true, 1.5)),
      loadTrajectory(new DriveTrajectory("BackChargeStation", false, 1.5)),
      m_drivetrain.setState(DrivetrainStates.AUTO_BALANCE));

  public final Command TWO_CONE_CHARGE_STATION = Auto(
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

  public final Command ONE_CONE_ONE_CUBE = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopNodeToTopCube", true)),
          INTAKE_CUBE()),
      loadTrajectory(new DriveTrajectory("TopCubeToTopCubeNode", false)),
      SCORE_CUBE(ShooterLevels.HIGH),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopCubeNodeToSecondPiece", false))));
  // INTAKE_CONE().beforeStarting(Commands.waitSeconds(2.2))),
  // HANDOFF());

  public final Command ONE_CONE_ONE_CUBE_BUMP = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("BottomPieceBumpSide", true, 1.5)),
          INTAKE_CUBE().beforeStarting(Commands.waitSeconds(2))),
      loadTrajectory(new DriveTrajectory("BottomPieceToBottomNode", false, 1.5)),
      SCORE_CUBE(ShooterLevels.HIGH),
      loadTrajectory(new DriveTrajectory("BottomNodeToOutside", false, 1.5)));

  public final Command ONE_CONE_ONE_CUBE_LOW_BUMP = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("BottomPieceBumpSide", true, 1.5)),
          INTAKE_CUBE().beforeStarting(Commands.waitSeconds(2))),
      loadTrajectory(new DriveTrajectory("BottomPieceToBottomNode", false, 1.5)),
      SCORE_CUBE(ShooterLevels.LOW),
      loadTrajectory(new DriveTrajectory("BottomNodeToOutside", false, 1.5)));

  public final Command ONE_CONE_PICKUP_CUBE_BUMP = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("BottomPieceBumpSide", true, 1.5)),
          INTAKE_CUBE().beforeStarting(Commands.waitSeconds(2))),
      loadTrajectory(new DriveTrajectory("BottomPieceToBottomNode", false, 1.5)));

  public final Command ONE_CONE_BUMP = Auto(
      SCORE_CONE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("BottomPieceBumpSide", true, 1.5)));

  private Command loadTrajectory(DriveTrajectory traj) {
    return Commands.waitUntil(m_drivetrain.isTrajectoryFinished)
        .deadlineWith(m_drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
            new HashMap<>(Map.of("Trajectory", traj.trajectory(), "IsFirstPath", traj.isFirstPath()))));
  }

  private Command Auto(Command... events) {
    return Commands.sequence(events)
        // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .finallyDo((i) -> System.out.println("Auto Routine Ended, Interrupted: " + i));
  }

  private final Command INIT() {
    return Commands.sequence(
        m_claw.setState(ClawStates.INTAKING),
        Intake.getInstance().setState(IntakeStates.RETRACTED),
        CubeShooter.getInstance().setState(CubeShooterStates.RETRACTED),
        Commands.waitSeconds(1),
        m_intake.setState(IntakeStates.RETRACTED),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  private final Command SCORE_CUBE(ElevatorStates extensionLevel) {
    return Commands.sequence(
        m_elevator.setState(extensionLevel),
        Commands.waitSeconds(0.75),
        m_wrist.setState(WristStates.DROPPING),
        Commands.waitSeconds(0.5),
        m_claw.setState(ClawStates.SPITTING),
        Commands.waitSeconds(0.2),
        m_wrist.setState(WristStates.RETRACTED),
        m_elevator.setState(ElevatorStates.RETRACT))
        .alongWith(INIT());
  }

  private final Command SCORE_CUBE(ShooterLevels shooterLevel) {
    return Commands.sequence(
        m_cubeShooter.Configure(shooterLevel),
        m_cubeShooter.setState(CubeShooterStates.SCORE),
        Commands.waitSeconds(0.5),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  private final Command SCORE_CONE(ElevatorStates extensionLevel) {
    return Commands.sequence(
        m_elevator.setState(extensionLevel),
        Commands.waitSeconds(0.75),
        m_wrist.setState(WristStates.DROPPING),
        new WaitCommand(0.15),
        m_claw.setState(ClawStates.SPITTING),
        new WaitCommand(0.2),
        m_elevator.setState(ElevatorStates.RETRACT),
        new WaitCommand(0.5),
        m_wrist.setState(WristStates.RETRACTED))
        .alongWith(INIT());
  }

  private final Command INTAKE_CONE() {
    return Commands.sequence(
        m_intake.setState(IntakeStates.INTAKE),
        Commands.waitSeconds(1.5),
        m_intake.setState(IntakeStates.HOLDING));
  }

  private final Command INTAKE_CUBE() {
    return Commands.sequence(
        m_cubeShooter.setState(CubeShooterStates.INTAKE),
        Commands.waitSeconds(2.75),
        m_cubeShooter.setState(CubeShooterStates.RETRACTED));
  }

  public final Command HANDOFF() {
    return Commands.sequence(
        Commands.deadline(
            Commands.sequence(
                m_intake.setState(IntakeStates.HANDOFF),
                m_claw.setState(ClawStates.INTAKING),
                m_wrist.setState(WristStates.HANDOFF_CONE),
                Commands.waitSeconds(0.35)),
            m_leds.SetStaticColor(Color.kFirebrick)).asProxy(),
        m_intake.setState(IntakeStates.SPIT),
        Commands.waitSeconds(0.1),
        m_wrist.setState(WristStates.RETRACTED),
        m_intake.setState(IntakeStates.RETRACTED));
  }
}
