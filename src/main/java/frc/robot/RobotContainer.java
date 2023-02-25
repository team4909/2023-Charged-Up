// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LED;

public class RobotContainer {

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final LED m_LED = LED.getInstance();
    public RobotContainer() {
        configureBindings();
        configureSendableChooser();
    }

    private void configureBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }

    private void configureSendableChooser() {
    }
}
