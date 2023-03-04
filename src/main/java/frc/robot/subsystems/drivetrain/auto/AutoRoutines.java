package frc.robot.subsystems.drivetrain.auto;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;

public class AutoRoutines {

  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final Elevator m_elevator = Elevator.getInstance();
  private final Arm m_arm = Arm.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Claw m_claw = Claw.getInstance();

  public final Auto ONE_METER_TEST = new Auto(
      loadTrajectory(new DriveTrajectory("Test", true)));
  public final Auto CHARGE_STATION = new Auto(
      loadTrajectory(new DriveTrajectory("ChargeStationStraight", true)));
  public final Auto SCORE_CONE_CHARGE_STATION_COMMUNITY = new Auto(
      SCORE_CONE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("ChargeStationStraight", true)),
      loadTrajectory(new DriveTrajectory("PastChargeStation", false)),
      loadTrajectory(new DriveTrajectory("BackChargeStation", false)),
      m_drivetrain.setState(DrivetrainStates.AUTO_BALANCE));
  public final Auto ONE_PIECE_CHARGE_STATION = new Auto(
      SCORE_CONE(ElevatorStates.TOP),
      loadTrajectory(new DriveTrajectory("TopNodeToTopPiece", true)),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("ThruTopPiece", false)),
          INTAKE_CONE()),
      Commands.parallel(
          loadTrajectory(new DriveTrajectory("TopPieceToTopNode", false)),
          HANDOFF()),
      SCORE_CONE(ElevatorStates.MID_CONE),
      loadTrajectory(new DriveTrajectory("TopNodeToChargeStation", false)));

  private Command loadTrajectory(DriveTrajectory traj) {
    return Commands.none().repeatedly().until(m_drivetrain.isTrajectoryFinished)
        .deadlineWith(m_drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
            new HashMap<>(
                Map.of("Trajectory", traj.getTrajectory(), "IsFirstPath", traj.getIsFirstPath()))));
  }

  private class Auto extends SequentialCommandGroup {
    public Auto(Command... events) {
      super.addCommands(events);
    }
  }

  private final Command SCORE_CONE(ElevatorStates extensionLevel) {
    return Commands.sequence(
        Commands.runOnce(() -> m_elevator.setState(extensionLevel)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> m_arm.setState(ArmStates.DROPPING)),
        Commands.waitSeconds(0.5),
        m_claw.setState(ClawStates.OPEN),
        Commands.waitSeconds(0.2),
        Commands.runOnce(() -> m_arm.setState(ArmStates.RETRACTED)),
        Commands.runOnce(() -> m_elevator.setState(ElevatorStates.RETRACT)));
  }

  private final Command INTAKE_CONE() {
    return Commands.sequence(
        m_intake.setState(IntakeStates.INTAKE_CONE),
        Commands.waitSeconds(2d),
        m_intake.setState(IntakeStates.HANDOFF));
  }

  public final Command HANDOFF() {
    return Commands.sequence(
        m_claw.setState(ClawStates.OPEN),
        Commands.runOnce(() -> m_arm.setState(ArmStates.HANDOFF_CONE)),
        Commands.waitSeconds(1),
        m_claw.setState(ClawStates.CLOSED),
        Commands.waitSeconds(1),
        m_intake.setState(IntakeStates.SPIT_CONE),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> m_arm.setState(ArmStates.RETRACTED)),
        m_intake.setState(IntakeStates.RETRACTED));
  }

}
