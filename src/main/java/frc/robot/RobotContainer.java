// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.auto.AutoRoutines;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final AutoRoutines m_routines = new AutoRoutines();

  private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public RobotContainer() {
    configureBindings();
    configureSendableChooser();
  }

  private void configureBindings() {
    m_drivetrain.setJoystickSuppliers(
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX());
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private void configureSendableChooser() {
    m_chooser.setDefaultOption("Test Auto", m_routines.testCommand());
    SmartDashboard.putData(m_chooser);
  }
}
