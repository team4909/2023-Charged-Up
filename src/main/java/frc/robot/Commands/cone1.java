package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsytem;


public class cone1 extends CommandBase {
  ElevatorSubsytem m_elev;
  int m_billy;

  public cone1(int setpoint) {
    m_billy = setpoint;
    m_elev = ElevatorSubsytem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elev);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elev.setSetpoint(m_billy);
     
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}