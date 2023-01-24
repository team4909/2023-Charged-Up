// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  Drivetrain m_drivetrain = Drivetrain.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setJoystickSuppliers(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
