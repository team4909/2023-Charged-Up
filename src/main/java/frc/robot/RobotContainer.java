// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.util.Color;
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
        // Temp button binds for cube/cone indication for human player
        m_driverController.a().onTrue(new InstantCommand(() -> m_LED.setLedColor(Color.kPurple)));
        m_driverController.b().onTrue(new InstantCommand(() -> m_LED.setLedColor(Color.kYellow)));
        
    }

    public Command getAutonomousCommand() {
        return null;
    }

    private void configureSendableChooser() {
    }
}
