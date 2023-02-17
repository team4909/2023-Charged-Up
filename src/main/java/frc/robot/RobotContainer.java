// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  private final Elevator m_elevator = Elevator.getInstance();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.b().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.TOP)));
    m_driverController.x().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CUBE)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CONE)));
    m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.RETRACT)));
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
