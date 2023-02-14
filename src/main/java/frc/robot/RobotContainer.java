// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.IntakeDeploy;

public class RobotContainer {
  
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //cone intake down and roller on
    m_driverController.rightTrigger().onTrue(new IntakeDeploy(12345).andThen(new RunIntakeRollers(0.5, -0.5)));
    //intake up and roller off
    m_driverController.rightTrigger().onFalse(new IntakeDeploy(42).andThen(new RunIntakeRollers(0, 0)));
    //Cube intake down and roller on
    m_driverController.rightBumper().onTrue(new IntakeDeploy(12345).andThen(new RunIntakeRollers(0.5, 0.5)));
    //intake up and roller off
    m_driverController.rightBumper().onFalse(new IntakeDeploy(42).andThen(new RunIntakeRollers(0, 0)));
    
    //intake down and spits cone
    m_driverController.leftTrigger().onTrue(new IntakeDeploy(12345).andThen(new RunIntakeRollers(-0.5, 0.5)));
    //Intake up and rollers off
    m_driverController.leftTrigger().onFalse(new IntakeDeploy(42).andThen(new RunIntakeRollers(0, 0)));
    //Intake down and spits cube
    m_driverController.leftBumper().onTrue(new IntakeDeploy(12345).andThen(new RunIntakeRollers(-0.5, 0.5)));
    //Intake up and rollers off
    m_driverController.leftBumper().onFalse(new IntakeDeploy(42).andThen(new RunIntakeRollers(0, 0)));
    // m_driverController.b().onTrue(new IntakeDeploy(28813));
    // m_driverController.y().onTrue(new IntakeDeploy(232));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0d)
        return (value - deadband) / (1d - deadband);
      else
        return (value + deadband) / (1d - deadband);
    } else {
      return 0d;
    }
  }

  private double modifyAxis(double value) {
    value = deadband(value, 0.05);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
}
