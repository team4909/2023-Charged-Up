// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmStates;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Claw.ClawStates;
import frc.robot.subsystems.drivetrain.DefaultDriveCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeStates;

public class RobotContainer {

  private final IntakeSubsystem m_intakeSubsytem = IntakeSubsystem.getInstance();

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private final Elevator m_elevator = Elevator.getInstance();

  private final Arm m_arm = Arm.getInstance();
  private final Claw m_claw = Claw.getInstance();
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();

  public RobotContainer() {
    configureBindings();

    m_drivetrain.setDefaultCommand(new DefaultDriveCommand(m_driverController));
  }

  private void configureBindings() {
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.TOP)));
    m_driverController.a().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.HANDOFF_CONE)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_arm.setState(ArmStates.HANDOFF_CUBE)));
    m_driverController.x().onTrue(new InstantCommand(() -> m_claw.setState(ClawStates.OPEN)));
    m_driverController.y().onTrue(new InstantCommand(() -> m_claw.setState(ClawStates.CLOSED)));

    m_driverController.leftTrigger()
        .onTrue(new RunCommand(() -> m_intakeSubsytem.cubeIn(), m_intakeSubsytem))
        .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));

    m_driverController.leftBumper()
        .onTrue(new RunCommand(() -> m_intakeSubsytem.cubeSpit(), m_intakeSubsytem))
        .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));

    m_driverController.rightTrigger()
        .onTrue(new RunCommand(() -> m_intakeSubsytem.coneIn(), m_intakeSubsytem))
        .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));

    m_driverController.rightBumper()
        .onTrue(new RunCommand(() -> m_intakeSubsytem.coneSpit(), m_intakeSubsytem))
        .onFalse(new RunCommand(() -> m_intakeSubsytem.handOff(), m_intakeSubsytem));
    m_driverController.x()
        .onTrue(new RunCommand(() -> m_intakeSubsytem.intakeIn(), m_intakeSubsytem));

    m_operatorController.b().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.TOP)));
    m_operatorController.x().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CUBE)));
    m_operatorController.y().onTrue(new InstantCommand(() -> m_elevator.setState(ElevatorStates.MID_CONE)));
    // m_operatorController.a().onTrue(new InstantCommand(() ->
    // m_elevator.setState(ElevatorStates.RETRACT)));

    // Mid Cone Sequence
    m_operatorController.a().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_claw.setState(ClawStates.OPEN)),
            new InstantCommand(() -> m_intakeSubsytem.handOff()),
            new InstantCommand(() -> m_arm.setState(ArmStates.HANDOFF_CONE)),
            new InstantCommand(() -> m_claw.setState(ClawStates.CLOSED)).withTimeout(3),
            new InstantCommand(() -> m_intakeSubsytem.coneSpit()),
            new InstantCommand(() -> m_arm.setState(ArmStates.TOP)))

    );
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
