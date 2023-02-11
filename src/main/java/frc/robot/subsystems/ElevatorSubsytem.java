// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsytem extends SubsystemBase {
  public TalonFX m_leadMotor;
  public TalonFX m_followMotor;

  public static final int leadDeviceID = 9;
  public static final int followDeviceID = 10;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  double m_leadMotorKp = .2;
  double m_leadMotorKI = 0;
  double m_leadMotorKD = 0.1;
  private static ElevatorSubsytem m_inst = null;

  double m_setpoint = 0;

  public static ElevatorSubsytem getInstance() {
    if (m_inst == null) {
      m_inst = new ElevatorSubsytem();
    }
    return m_inst;
  }

  /** Creates a new ElevatorSubsytem. */
  private ElevatorSubsytem() {
    m_leadMotor = new TalonFX(leadDeviceID, "CANivore1");
    m_followMotor = new TalonFX(followDeviceID, "CANivore1");

    m_leadMotor.configFactoryDefault();
    m_followMotor.configFactoryDefault();

    m_followMotor.follow(m_leadMotor);
    m_followMotor.setInverted(true);

    m_leadMotor.config_kP(0, m_leadMotorKp);
    m_leadMotor.config_kI(0, m_leadMotorKI);
    m_leadMotor.config_kD(0, m_leadMotorKD);

    m_leadMotor.configClosedLoopPeakOutput(0, .5);
    // m_leadMotor.configClosedloopRamp()

    m_leadMotor.set(TalonFXControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevatorSetpoint", m_setpoint);
    SmartDashboard.putNumber("elevatorError", m_leadMotor.getClosedLoopError());
    SmartDashboard.putNumber("elevatorEncoder", m_leadMotor.getSelectedSensorPosition());
  }

  public void setSetpoint(double distance) {
   m_leadMotor.set(TalonFXControlMode.Position, distance);
   m_setpoint = distance;
  }
}
