package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Intake.IntakeSubsystem;

public class IntakeDeploy  extends CommandBase  {
   IntakeSubsystem m_intake; 
    int m_setpoint;

    public IntakeDeploy(int setpoint){
        m_setpoint = setpoint;
        m_intake = IntakeSubsystem.getInstance();
        
        addRequirements(m_intake);
    }

    @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_intake.setWristSetpoint(m_setpoint);
     
  
  }
  @Override
  public void end(boolean interrupted) {}
 
 
  @Override
  public boolean isFinished() {
    return true;
  }
}


