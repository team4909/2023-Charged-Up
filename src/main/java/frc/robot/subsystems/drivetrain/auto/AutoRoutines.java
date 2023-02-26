package frc.robot.subsystems.drivetrain.auto;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;

public class AutoRoutines {

    private final Drivetrain m_drivetrain = Drivetrain.getInstance();
    private final Elevator m_elevator = Elevator.getInstance();
    private final Arm m_arm = Arm.getInstance();
    private final Claw m_claw = Claw.getInstance();

    public final Auto CHARGE_STATION = new Auto(
            loadTrajectory(new DriveTrajectory(new Pair<String, Double>("ChargeStationStraight", 2d), true)));
    public final Auto SCORE_CONE_CHARGE_STATION_COMMUNITY = new Auto(
            SCORE_CONE_HIGH(),
            loadTrajectory(new DriveTrajectory(new Pair<String, Double>("ChargeStationStraight", 2d), true)),
            loadTrajectory(new DriveTrajectory(new Pair<String, Double>("PastChargeStation", 2d), false)),
            loadTrajectory(new DriveTrajectory(new Pair<String, Double>("BackChargeStation", 2d), false)));

    // private Command loadTrajectory(DriveTrajectory traj) {
    // return m_drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
    // new HashMap<>(Map.of("Trajectory", traj.getTrajectory(), "Timeout",
    // traj.getTimeout())))
    // .deadlineWith(new
    // InstantCommand().repeatedly().until(m_drivetrain.isTrajectoryFinished))
    // .finallyDo((e) -> System.out.println("What are you doing " + e));
    // }

    private Command loadTrajectory(DriveTrajectory traj) {
        return new InstantCommand().repeatedly().until(m_drivetrain.isTrajectoryFinished)
                .deadlineWith(m_drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
                        new HashMap<>(Map.of("Trajectory", traj.getTrajectory(), "Timeout", traj.getTimeout()))));
    }

    private class Auto extends SequentialCommandGroup {
        public Auto(Command... events) {
            super.addCommands(events);
        }
    }

    private final Command SCORE_CONE_HIGH() {
        return Commands.sequence(
                Commands.runOnce(() -> m_elevator.setState(ElevatorStates.TOP)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> m_arm.setState(ArmStates.DROPPING)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> m_claw.setState(ClawStates.OPEN)),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> m_arm.setState(ArmStates.TOP)),
                Commands.runOnce(() -> m_elevator.setState(ElevatorStates.RETRACT)),
                Commands.waitSeconds(3));
    }
}
