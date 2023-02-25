package frc.robot.subsystems.drivetrain.auto;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;

public class AutoRoutines {

    private final Drivetrain drivetrain = Drivetrain.getInstance();
    public final Auto CHARGE_STATION = new Auto(List.of(
            loadTrajectory(new DriveTrajectory(new Pair<String, Double>("ChargeStationStraight", 5d), true))),
            drivetrain);

    private Command loadTrajectory(DriveTrajectory traj) {
        return drivetrain.setState(DrivetrainStates.TRAJECTORY_DRIVE,
                new HashMap<>(Map.of("Trajectory", traj.getTrajectory(), "Timeout", traj.getTimeout())));
    }

    private class Auto extends SequentialCommandGroup {
        public Auto(List<Command> events, Subsystem... requirements) {
            Command[] commandArr = new Command[events.size()];
            super.addCommands(events.toArray(commandArr));
            super.addRequirements(requirements);
        }
    }
}
