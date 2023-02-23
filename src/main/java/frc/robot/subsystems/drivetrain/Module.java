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
    private TalonFX m_driveMotor;
    private TalonFX m_turnMotor;
    private double m_offset ; 
    private CANCoder m_enc;
    private String m_name;
    // ShuffleboardTab tab;

    public Module(String name, int driveMotorCanId, int turnMotorCanId, int encoderCanId, double encOffset) {
        m_name = name;
        m_offset = encOffset;

        m_driveMotor = new TalonFX(driveMotorCanId, "CANivore1"); 
        m_turnMotor = new TalonFX(turnMotorCanId, "CANivore1");
        m_enc = new CANCoder(encoderCanId, "CANivore1");

        m_enc.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_turnMotor.configFactoryDefault();
        m_driveMotor.configFactoryDefault();

        double currentHeadingTicks = -1 * Drivetrain.convertDegreesToTicks(getHeading());
        m_turnMotor.setSelectedSensorPosition(currentHeadingTicks);
        m_turnMotor.set(ControlMode.Position, currentHeadingTicks);
        SmartDashboard.putNumber(m_name + " initial heading", getHeading());

        m_turnMotor.setInverted(true);

        double turnMotorKp = .2;
        double turnMotorKI = 0;
        double turnMotorKD = 0.1;

        double driveMotorkP = 0.05;
        double driveMotorkI = 0.0;
        double driveMotorkD = 0.0;

        m_turnMotor.config_kP(0, turnMotorKp);
        m_turnMotor.config_kI(0, turnMotorKI);
        m_turnMotor.config_kD(0, turnMotorKD);
        m_turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

        m_driveMotor.config_kP(0, driveMotorkP);
        m_driveMotor.config_kI(0, driveMotorkI);
        m_driveMotor.config_kD(0, driveMotorkD);

        m_driveMotor.setInverted(false);
        m_driveMotor.setNeutralMode(NeutralMode.Brake);

    }
  
    public double getHeading() {
        double encoderReading = Math.floorMod((int) m_enc.getPosition(), 360);
        
       return encoderReading - m_offset;
    }


    public void periodic() {
        SmartDashboard.putNumber(m_name + " Cancoder",  m_enc.getPosition());
        SmartDashboard.putNumber(m_name + " Cancoder with offset",  getHeading());
        SmartDashboard.putNumber(m_name + " error", m_turnMotor.getClosedLoopError());
        SmartDashboard.putNumber(m_name + " error deg", Drivetrain.convertTicksToDegrees(m_turnMotor.getClosedLoopError()));
        SmartDashboard.putNumber(m_name + " m_turnMotor", Drivetrain.convertTicksToDegrees(m_turnMotor.getSelectedSensorPosition()));
    }

    public void setModuleState(SwerveModuleState desiredState) {

        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getHeading()));

        SmartDashboard.putNumber(m_name + " Heading", desiredState.angle.getDegrees());

        double desiredMetersPerSecond = (desiredState.speedMetersPerSecond / Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        SmartDashboard.putNumber(m_name + " desiredMetersPerSecond", desiredMetersPerSecond);

        m_driveMotor.set(ControlMode.Velocity, desiredMetersPerSecond / getDriveMotorReduction());
        m_turnMotor.set(ControlMode.Position, Drivetrain.convertDegreesToTicks(desiredState.angle.getDegrees()));
    }

    public SwerveModuleState getState() {

        double speedTicksPer100miliSeconds = m_driveMotor.getSelectedSensorVelocity();

        double speedMetersPerSecond = speedTicksPer100miliSeconds * getDriveMotorReduction();

        SmartDashboard.putNumber(m_name + "St", speedTicksPer100miliSeconds);
        SmartDashboard.putNumber(m_name + "Sm", speedMetersPerSecond);

        return new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees( getHeading()));

    }
    public SwerveModulePosition getPosition () {
        // 2048 ticks per rotation
        // 6.75:1 gear ratio
        // 4" wheels
        double distanceMeters = (1 / 2048d) * (1 / 6.75) * ((4.0 * Math.PI) / 1d) * (.0254 / 1) * m_driveMotor.getSelectedSensorPosition();
        Rotation2d angle = Rotation2d.fromDegrees( getHeading());
        return new SwerveModulePosition(distanceMeters, angle);
    }
    public void resetTurnEncoders() {
        // m_enc.setPosition(0);
        // m_turnMotor.setSelectedSensorPosition(DrivetrainSubsystem.convertDegreesToTicks(0));
        m_turnMotor.set(ControlMode.Position, 0);
    }

    /**
     * @return Ticks of the drive motor per revolution of the wheel
     */
    private double getDriveMotorReduction(){
        return (1 / 100d) * (1_000 / 1d) * (1 / 2048d) * (1 / 6.75) * ((4.0 * Math.PI) / 1d) * (.0254 / 1);
    }
}