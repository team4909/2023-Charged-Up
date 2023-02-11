package frc.robot.subsystems.drivetrain.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoRoutines {
    
    public ParallelCommandGroup testCommand() {
        return new ParallelCommandGroup(
          new TrajectoryFollow("Test")  
        );
    }
}
