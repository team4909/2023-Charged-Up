// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Wrist.WristStates;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivetrain.DrivetrainStates;
import frc.robot.subsystems.intake.CubeShooter;
import frc.robot.subsystems.intake.CubeShooter.CubeShooterStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.leds.LEDs;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private PDH m_pdh;

  public Robot() {
    m_robotContainer = new RobotContainer();
    // m_pdh = new PDH();
    addPeriodic(m_robotContainer.controlLoop(), 0.01, 0.005);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotInit() {
    Drivetrain.getInstance().reseedModules();
  }

  @Override
  public void robotPeriodic() {
    // m_pdh.periodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    LEDs.getInstance().setDefaultCommand(LEDs.getInstance().setBreatheColor(new Color(0, 255, 0)));
  }

  @Override
  public void disabledPeriodic() {
    Drivetrain.getInstance().reseedModules();
  }

  @Override
  public void disabledExit() {
    m_robotContainer.initLEDS();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Intake.getInstance().setState(IntakeStates.RETRACTED).schedule();
    Wrist.getInstance().setState(WristStates.RETRACTED).schedule();
    CubeShooter.getInstance().setState(CubeShooterStates.RETRACTED).schedule();
    Drivetrain.getInstance().setState(DrivetrainStates.IDLE).schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
    DriverStation.silenceJoystickConnectionWarning(true);
    SimVisualizer.getInstance();

  }

  @Override
  public void simulationPeriodic() {

  }
}
