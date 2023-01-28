package frc.robot.subsystems.drivetrain.module;

import static frc.robot.Constants.DrivetrainConstants.MODULE_CONFIGURATION;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

public final class PhysicalModule extends ModuleBase {

    private final double TICKS_PER_ROTATION = 2048d;
    private final TalonFX m_driveMotor;
    private final TalonFX m_turnMotor;
    private final CANCoder m_encoder;
    private final Rotation2d m_encoderOffset;

    private final double nominalVoltage = 12d;

    public PhysicalModule(int index) {
        switch (index) {
            case 0:
                m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.FRONT_LEFT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = new Rotation2d(DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
                break;
            case 1:
                m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.FRONT_RIGHT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = new Rotation2d(DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
                break;
            case 2:
                m_driveMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.BACK_LEFT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = new Rotation2d(DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
                break;
            case 3:
                m_driveMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.BACK_RIGHT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = new Rotation2d(DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
                break;
            default:
                throw new RuntimeException("Invalid Module index");
        }

        configHardware();
    }

    @Override
    void updateModuleInputs() {
        super.drivePositionRad = Units.rotationsToRadians(m_driveMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION)
                * MODULE_CONFIGURATION.getDriveReduction();
        super.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                m_driveMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
                * MODULE_CONFIGURATION.getDriveReduction();
        super.driveCurrentAmps = m_driveMotor.getSupplyCurrent();
        super.driveAppliedVolts = m_driveMotor.getMotorOutputVoltage();

        super.turnPositionRad = Units.rotationsToRadians(m_turnMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION)
                * MODULE_CONFIGURATION.getSteerReduction();
        super.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                m_turnMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
                * MODULE_CONFIGURATION.getSteerReduction();
        super.turnCurrentAmps = m_turnMotor.getSupplyCurrent();
        super.turnAppliedVolts = m_turnMotor.getMotorOutputVoltage();

        double angle = Math.toRadians(m_encoder.getAbsolutePosition()) % (2d * Math.PI);
        if (angle < 0d)
            angle += 2d * Math.PI;
        super.turnAbsolutePositionRad = new Rotation2d(Math.toRadians(m_encoder.getAbsolutePosition()))
                .minus(m_encoderOffset).getRadians();
    }

    @Override
    void setDriveVolts(double volts) {
        m_driveMotor.set(TalonFXControlMode.PercentOutput, volts / nominalVoltage);
    }

    @Override
    void setTurnVolts(double volts) {
        SmartDashboard.putNumber("percent", volts);
        m_turnMotor.set(TalonFXControlMode.PercentOutput, volts / nominalVoltage);
    }

    private void configHardware() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.setSelectedSensorPosition(0);
        m_turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 1));
        m_driveMotor.configVoltageCompSaturation(12);
        m_driveMotor.enableVoltageCompensation(true);
        m_driveMotor.setInverted(MODULE_CONFIGURATION.isDriveInverted());

        m_turnMotor.configFactoryDefault();
        m_turnMotor.setSelectedSensorPosition(0);
        m_turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 1));
        m_turnMotor.configVoltageCompSaturation(12);
        m_turnMotor.enableVoltageCompensation(true);
        m_turnMotor.setInverted(MODULE_CONFIGURATION.isSteerInverted());

        m_encoder.configFactoryDefault();
        m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

}