package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DefaultDriveCommand extends CommandBase {

    private final Drivetrain m_drivetrainSubsystem;

    CommandXboxController m_driver;

    public DefaultDriveCommand(CommandXboxController driver) {
        m_drivetrainSubsystem = Drivetrain.getInstance();
        m_driver = driver;

        this.addRequirements(m_drivetrainSubsystem);
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.DriveWithJoystick(m_driver);
    }

    
    @Override
    public void end(boolean interrupted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}