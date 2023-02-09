package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public final class PhysicalModule extends ModuleBase {

    private final TalonFX m_driveMotor;
    private final TalonFX m_turnMotor;
    private final CANCoder m_encoder;
    private final double m_encoderOffset;

    private final double TICKS_PER_ROTATION = 2048d;
    private final double nominalVoltage = 12d;

    public PhysicalModule(int index) {
        switch (index) {
            case 0:
                m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.FRONT_LEFT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
                break;
            case 1:
                m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.FRONT_RIGHT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
                break;
            case 2:
                m_driveMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.BACK_LEFT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
                break;
            case 3:
                m_driveMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_DRIVE_MOTOR, DrivetrainConstants.CANBUS);
                m_turnMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_TURN_MOTOR, DrivetrainConstants.CANBUS);
                m_encoder = new CANCoder(DrivetrainConstants.BACK_RIGHT_STEER_ENCODER, DrivetrainConstants.CANBUS);
                m_encoderOffset = DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
                break;
            default:
                throw new RuntimeException("Invalid Module index");
        }

        configHardware();
    }

    @Override
    void updateModuleInputs() {
        super.drivePositionRad = Units.rotationsToRadians(m_driveMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION)
                * DrivetrainConstants.DRIVE_REDUCTION;
        super.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                m_driveMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
                * DrivetrainConstants.DRIVE_REDUCTION;
        super.driveCurrentAmps = m_driveMotor.getSupplyCurrent();
        super.driveAppliedVolts = m_driveMotor.getMotorOutputVoltage();

        super.turnPositionRad = Units.rotationsToRadians(m_turnMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION)
                * DrivetrainConstants.STEER_REDUCTION;
        super.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                m_turnMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
                * DrivetrainConstants.STEER_REDUCTION;
        super.turnCurrentAmps = m_turnMotor.getSupplyCurrent();
        super.turnAppliedVolts = m_turnMotor.getMotorOutputVoltage();
        super.turnAbsolutePosition = getWheelHeading();
    }

    @Override
    void setDriveVolts(double volts) {
        m_driveMotor.set(TalonFXControlMode.PercentOutput, volts / nominalVoltage);
    }

    @Override
    void setTurn(double pos) {
        m_turnMotor.set(TalonFXControlMode.Position, pos);
    }

    private void configHardware() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 1));
        m_driveMotor.configVoltageCompSaturation(12);
        m_driveMotor.enableVoltageCompensation(true);
        m_driveMotor.setInverted(false);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);
        m_driveMotor.setSelectedSensorPosition(0);

        m_turnMotor.configFactoryDefault();
        m_turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 1));
        m_turnMotor.configVoltageCompSaturation(12);
        m_turnMotor.enableVoltageCompensation(true);
        m_turnMotor.setInverted(true);
        m_turnMotor.setNeutralMode(NeutralMode.Brake);
        m_turnMotor.setSelectedSensorPosition(-1 * Module.convertDegreesToTicks(getWheelHeading()));
        m_turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

        m_turnMotor.config_kP(0, 0.2);
        m_turnMotor.config_kD(0, 0.1);

        m_encoder.configFactoryDefault();
        m_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    private double getWheelHeading() {
        return Math.floorMod((int) m_encoder.getPosition(), 360) - m_encoderOffset;
    }

}