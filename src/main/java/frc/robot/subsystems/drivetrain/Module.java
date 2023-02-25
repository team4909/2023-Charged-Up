package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder; 
import com.ctre.phoenix.sensors.SensorInitializationStrategy; 

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Module {
    TalonFX driveMotor;
    TalonFX turnMotor;
    double m_offset ; 
    CANCoder enc;
    String m_name;
    // ShuffleboardTab tab;

    public Module(String name, int driveMotorCanId, int turnMotorCanId, int encoderCanId, double encOffset) {
        this.m_name = name;
        m_offset = encOffset;
        // this.tab = DrivetrainSubsystem.getInstance().getTab();

        driveMotor = new TalonFX(driveMotorCanId, "CANivore1"); 
        turnMotor = new TalonFX(turnMotorCanId, "CANivore1");
        enc = new CANCoder(encoderCanId, "CANivore1");

        enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // enc.setStatusFramePeriod(statusFrame, periodMs)

        turnMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();
        // set motor encoder to 0 when robot code starts

        var currentHeadingTicks = -1 * Drivetrain.convertDegreesToTicks(getHeading());
        turnMotor.setSelectedSensorPosition(currentHeadingTicks);
        turnMotor.set(ControlMode.Position, currentHeadingTicks);
        SmartDashboard.putNumber(m_name + " initial heading", getHeading());

        turnMotor.setInverted(true);

        double turnMotorKp = .2;
        double turnMotorKI = 0;
        double turnMotorKD = 0.1;

        turnMotor.config_kP(0, turnMotorKp);
        turnMotor.config_kI(0, turnMotorKI);
        turnMotor.config_kD(0, turnMotorKD);
        turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
        //turnMotor.setNeutralMode(NeutralMode.Brake);


        driveMotor.setInverted(false);
        driveMotor.setNeutralMode(NeutralMode.Brake);

    }
  
    public double getHeading() {
        double v = Math.floorMod((int)enc.getPosition(), 360);
        
       return v - m_offset;
    }


    public void periodic() {
        SmartDashboard.putNumber(m_name + " Cancoder",  enc.getPosition());
        SmartDashboard.putNumber(m_name + " Cancoder with offset",  getHeading());
        SmartDashboard.putNumber(m_name + " error", turnMotor.getClosedLoopError());
        SmartDashboard.putNumber(m_name + " error deg", Drivetrain.convertTicksToDegrees(turnMotor.getClosedLoopError()));
        SmartDashboard.putNumber(m_name + " turnMotor", Drivetrain.convertTicksToDegrees(turnMotor.getSelectedSensorPosition()));
    }

    public final double MAX_VOLTAGE = 8;
    public final double NOMINAL_VOLTAGE = 12;

    public void setModuleState(SwerveModuleState desiredState) {

        // tab.add(name + " cancoder", getHeading());
        // tab.add(name + " cancoder",  getHeading());
        
        

        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getHeading()));

        SmartDashboard.putNumber(m_name + " Heading", desiredState.angle.getDegrees());

        double desiredPercentOutput = (desiredState.speedMetersPerSecond / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND) * (MAX_VOLTAGE / NOMINAL_VOLTAGE);
        SmartDashboard.putNumber(m_name + " desiredVoltage", desiredPercentOutput);

        driveMotor.set(ControlMode.PercentOutput, desiredPercentOutput);
        turnMotor.set(ControlMode.Position, Drivetrain.convertDegreesToTicks(desiredState.angle.getDegrees()));
    }

    public SwerveModuleState getState() {

        double speedTicksPer100miliSeconds = driveMotor.getSelectedSensorVelocity();

        double speedMetersPerSecond = speedTicksPer100miliSeconds * (1 / 100d) * (1_000 / 1d) * (1 / 2048d) * (1 / 6.75)
                * ((4.0 * Math.PI) / 1d) * (.0254 / 1);

        SmartDashboard.putNumber(m_name + "St", speedTicksPer100miliSeconds);
        SmartDashboard.putNumber(m_name + "Sm", speedMetersPerSecond);

        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees( getHeading()));

    }
    public SwerveModulePosition getPosition () {
        // 2048 ticks per rotation
        // 6.75:1 gear ratio
        // 4" wheels
        double distanceMeters = (1 / 2048d) * (1 / 6.75) * ((4.0 * Math.PI) / 1d) * (.0254 / 1) * driveMotor.getSelectedSensorPosition();
        Rotation2d angle = Rotation2d.fromDegrees( getHeading());
        ;
        return new SwerveModulePosition(distanceMeters, angle);
    }
    public void resetTurnEncoders() {
        // enc.setPosition(0);
        // turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(0));
        turnMotor.set(ControlMode.Position, 0);
    }
}