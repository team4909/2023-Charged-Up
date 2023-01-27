package frc.robot.subsystems.drivetrain.module;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.DrivetrainConstants;

public final class PhysicalModule extends ModuleBase {

    private final String CANBUS = "Drivetrain-CANivore";
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANCoder encoder;

    private final double nominalVoltage = 12d;

    public PhysicalModule(int index) {
        switch (index) {
            case 0:
                driveMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_DRIVE_MOTOR, CANBUS);
                turnMotor = new TalonFX(DrivetrainConstants.FRONT_LEFT_TURN_MOTOR, CANBUS);
                encoder = new CANCoder(DrivetrainConstants.FRONT_LEFT_STEER_ENCODER, CANBUS);
                break;
            case 1:
                driveMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_DRIVE_MOTOR, CANBUS);
                turnMotor = new TalonFX(DrivetrainConstants.FRONT_RIGHT_TURN_MOTOR, CANBUS);
                encoder = new CANCoder(DrivetrainConstants.FRONT_RIGHT_STEER_ENCODER, CANBUS);
                break;
            case 2:
                driveMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_DRIVE_MOTOR, CANBUS);
                turnMotor = new TalonFX(DrivetrainConstants.BACK_LEFT_TURN_MOTOR, CANBUS);
                encoder = new CANCoder(DrivetrainConstants.BACK_LEFT_STEER_ENCODER, CANBUS);
                break;
            case 3:
                driveMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_DRIVE_MOTOR, CANBUS);
                turnMotor = new TalonFX(DrivetrainConstants.BACK_RIGHT_TURN_MOTOR, CANBUS);
                encoder = new CANCoder(DrivetrainConstants.BACK_RIGHT_STEER_ENCODER, CANBUS);
                break;
            default:
                throw new RuntimeException("Invalid Module index");
        }

        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        encoder.configFactoryDefault();
    }

    @Override
    void updateModuleInputs() {
        super.driveCurrentAmps = driveMotor.getSupplyCurrent();

        super.turnCurrentAmps = turnMotor.getSupplyCurrent();
    }

    @Override
    void setDriveVolts(double volts) {
        driveMotor.set(TalonFXControlMode.PercentOutput, volts / nominalVoltage);
    }

    @Override
    void setTurnVolts(double volts) {
        turnMotor.set(TalonFXControlMode.PercentOutput, volts / nominalVoltage);
    }

}