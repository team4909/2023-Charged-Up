package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.util.Units;
import frc.lib.bioniclib.CANConfigurator;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public final class PhysicalModule extends ModuleBase {

  private final int m_index;
  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;
  private final CANCoder m_encoder;
  private final double m_encoderOffset;

  private final double TICKS_PER_ROTATION = 2048d;

  public PhysicalModule(int index) {
    m_index = index;
    switch (index) {
      case 0:
        m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_DRIVE_MOTOR, Constants.CANFD_BUS);
        m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_TURN_MOTOR, Constants.CANFD_BUS);
        m_encoder = new CANCoder(DrivetrainConstants.FRONT_LEFT_STEER_ENCODER, Constants.CANFD_BUS);
        m_encoderOffset = DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
        break;
      case 1:
        m_driveMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_DRIVE_MOTOR, Constants.CANFD_BUS);
        m_turnMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_TURN_MOTOR, Constants.CANFD_BUS);
        m_encoder = new CANCoder(DrivetrainConstants.FRONT_RIGHT_STEER_ENCODER, Constants.CANFD_BUS);
        m_encoderOffset = DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
        break;
      case 2:
        m_driveMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_DRIVE_MOTOR, Constants.CANFD_BUS);
        m_turnMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_TURN_MOTOR, Constants.CANFD_BUS);
        m_encoder = new CANCoder(DrivetrainConstants.BACK_LEFT_STEER_ENCODER, Constants.CANFD_BUS);
        m_encoderOffset = DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
        break;
      case 3:
        m_driveMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_DRIVE_MOTOR, Constants.CANFD_BUS);
        m_turnMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_TURN_MOTOR, Constants.CANFD_BUS);
        m_encoder = new CANCoder(DrivetrainConstants.BACK_RIGHT_STEER_ENCODER, Constants.CANFD_BUS);
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

    // super.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
    // m_driveMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
    // * DrivetrainConstants.DRIVE_REDUCTION;

    super.driveVelocityRadPerSec = Module.convertTicksToMPS(m_driveMotor.getSelectedSensorVelocity());

    // super.driveCurrentAmps = m_driveMotor.getSupplyCurrent();
    // super.driveAppliedVolts = m_driveMotor.getMotorOutputVoltage();

    // super.turnPositionRad =
    // Units.rotationsToRadians(m_turnMotor.getSelectedSensorPosition() /
    // TICKS_PER_ROTATION)
    // * DrivetrainConstants.STEER_REDUCTION;

    super.turnPosition = Module.convertTicksToDegrees(m_turnMotor.getSelectedSensorPosition());
    super.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        m_turnMotor.getSelectedSensorVelocity() * (600 / TICKS_PER_ROTATION))
        * DrivetrainConstants.STEER_REDUCTION;

    // super.turnCurrentAmps = m_turnMotor.getSupplyCurrent();
    // super.turnAppliedVolts = m_turnMotor.getMotorOutputVoltage();

    super.turnAbsolutePositionRad = 0;
  }

  @Override
  void setDrive(double input, double ff) {
    m_driveMotor.set(TalonFXControlMode.Velocity, input, DemandType.ArbitraryFeedForward,
        ff);
  }

  @Override
  void setTurn(double pos) {
    m_turnMotor.set(TalonFXControlMode.Position, pos);
  }

  private void configHardware() {
    CANConfigurator<ErrorCode> moduleManager = new CANConfigurator<>(
        "Module: " + Module.matchModuleName(m_index), ErrorCode.class);
    moduleManager.actionConsumer.accept(() -> m_driveMotor.configFactoryDefault(Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(
        () -> m_driveMotor
            .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0),
                Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(
        () -> m_driveMotor
            .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0),
                Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_driveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms,
            Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_driveMotor.configVelocityMeasurementWindow(32, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_driveMotor.configVoltageCompSaturation(12, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_driveMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_driveMotor.config_kP(0, DrivetrainConstants.DRIVE_kP, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_driveMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General,
        250,
        Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_driveMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0,
        5,
        Constants.kCANTimeoutMs));
    m_driveMotor.enableVoltageCompensation(true);
    m_driveMotor.setInverted(false);
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_driveMotor.setSensorPhase(true);

    moduleManager.actionConsumer.accept(() -> m_turnMotor.configFactoryDefault(Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(
        () -> m_turnMotor
            .configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0)));
    moduleManager.actionConsumer.accept(
        () -> m_turnMotor
            .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0)));
    moduleManager.actionConsumer.accept(() -> m_turnMotor.configVoltageCompSaturation(12, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_turnMotor.setSelectedSensorPosition(Module.convertDegreesToTicks(getWheelHeading()), 0,
            Constants.kCANTimeoutMs));
    moduleManager.actionConsumer
        .accept(() -> m_turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360));
    moduleManager.actionConsumer.accept(() -> m_turnMotor.config_kP(0, DrivetrainConstants.TURN_kP, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_turnMotor.config_kD(0, DrivetrainConstants.TURN_kD, Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_turnMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General,
        5,
        Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_turnMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0,
        5,
        Constants.kCANTimeoutMs));
    m_turnMotor.enableVoltageCompensation(true);
    m_turnMotor.setInverted(true);
    m_turnMotor.setNeutralMode(NeutralMode.Brake);
    m_turnMotor.setSensorPhase(false);

    moduleManager.actionConsumer.accept(() -> m_encoder.configFactoryDefault(Constants.kCANTimeoutMs));
    moduleManager.actionConsumer.accept(() -> m_encoder
        .configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition));
    moduleManager.actionConsumer
        .accept(() -> m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360));
    moduleManager.forceConfig();
  }

  private double getWheelHeading() {
    return m_encoder.getAbsolutePosition() - m_encoderOffset;
  }

  void resetTurn() {
    m_turnMotor.setSelectedSensorPosition(Module.convertDegreesToTicks(getWheelHeading()));
  }

}
