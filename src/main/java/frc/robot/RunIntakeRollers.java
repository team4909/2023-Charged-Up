package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake.IntakeSubsystem;

public class RunIntakeRollers extends CommandBase {
  IntakeSubsystem m_intake;
  double m_frontRollerSpeed;
  double m_backRollerSpeed;
  
  
  public RunIntakeRollers(double frontRollerSpeed, double backRollerSpeed){
    m_intake = IntakeSubsystem.getInstance();
    m_frontRollerSpeed = frontRollerSpeed;
    m_backRollerSpeed = backRollerSpeed; 
    
    addRequirements(m_intake);
  } 



    @Override
  public void initialize() {}

  

  @Override
  public void execute() {
    m_intake.setRollerSpeed(m_frontRollerSpeed, m_backRollerSpeed);
  }

  @Override
  public void end(boolean interrupted) {}



  @Override
  public boolean isFinished() {
    return true;
  }
}
