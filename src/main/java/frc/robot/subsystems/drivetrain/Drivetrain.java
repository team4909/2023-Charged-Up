package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swervelib.Mk4ModuleConfiguration;
import frc.lib.swervelib.Mk4iSwerveModuleHelper;
import frc.lib.swervelib.SwerveModule;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain m_instance = null;
    private DrivetrainStates m_state;
    private DrivetrainStates m_lastState;

    private ChassisSpeeds m_chassisSpeeds;
    private DoubleSupplier m_joystickTranslationX;
    private DoubleSupplier m_joystickTranslationY;
    private DoubleSupplier m_joystickRotationOmega;
    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;
    private SwerveModuleState[] m_states = new SwerveModuleState[4];
    private SwerveDriveOdometry m_odometry;
    private Pose2d m_pose;
    private Field2d m_field;

    private final Pigeon2 m_pigeon = new Pigeon2(PIGEON_ID, "Drivetrain-CANivore");
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // FL
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // FR
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // BL
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // BR
    );

    private final double MAX_VOLTAGE = 12d; 
    private final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_500_FREE_SPEED / 60.0 *
            MODULE_CONFIGURATION.getDriveReduction() *
            MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;

    private enum DrivetrainStates {
        JOYSTICK_DRIVE("Joystick Drive"),
        PRECISE("Precise"),
        LOCKED("Locked");

        String stateName;

        private DrivetrainStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return this.stateName;
        }
    }

    private Drivetrain() {
        initializeMotors();
        m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroYaw(), getSwerveModulePositions());
        m_states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        m_field = new Field2d();
        m_lastState = null; //@TODO these should be the same and should be an IDLE state
        m_state = DrivetrainStates.JOYSTICK_DRIVE; //Initial State
    }

    public void periodic() {
        stateMachine();
        updateSwerveModuleStates();
        m_frontLeftModule.set(m_states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, m_states[0].angle.getRadians());
        m_frontRightModule.set(m_states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, m_states[1].angle.getRadians());
        m_backLeftModule.set(m_states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, m_states[2].angle.getRadians());
        m_backRightModule.set(m_states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, m_states[3].angle.getRadians());
        m_field.setRobotPose(m_pose);
        if (Constants.SIM)
            m_pose = m_odometry.update(getGyroYaw(), getSwerveModulePositions());
        else
            m_pose = m_odometry.update(getGyroYaw(), getSwerveModulePositions());
    }

    public void simulationPeriodic() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putNumber("suppler x", m_joystickTranslationX.getAsDouble());
        SmartDashboard.putString("State", m_state.toString());
        SmartDashboard.putString("module 1", m_states[0].toString());
        SmartDashboard.putString("chassis speeds", m_chassisSpeeds.toString());
    }


    private void initializeMotors() {
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();
        config.setCanivoreName("Drivetrain-CANivore");
        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                config,
                GEAR_RATIO,
                FRONT_LEFT_DRIVE_MOTOR,
                FRONT_LEFT_STEER_MOTOR,
                FRONT_LEFT_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);
        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                config,
                GEAR_RATIO,
                FRONT_RIGHT_DRIVE_MOTOR,
                FRONT_RIGHT_STEER_MOTOR,
                FRONT_RIGHT_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET);
        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                config,
                GEAR_RATIO,
                BACK_LEFT_DRIVE_MOTOR,
                BACK_LEFT_STEER_MOTOR,
                BACK_LEFT_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET);
        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                config,
                GEAR_RATIO,
                BACK_RIGHT_DRIVE_MOTOR,
                BACK_RIGHT_STEER_MOTOR,
                BACK_RIGHT_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET);
        m_pigeon.clearStickyFaults();
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    // *Current* Positions
    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), m_backRightModule.getPosition()
        };
    }

    // *Desired* States
    private void updateSwerveModuleStates() {
        m_states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);
    }

    public void setJoystickSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_joystickTranslationX = x;
        m_joystickTranslationY = y;
        m_joystickRotationOmega = omega;
    }

    private void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    private void stateMachine() {
        Command currentDrivetrainCommand = null;
        if (!m_state.equals(m_lastState)) {
            switch (m_state) {
                case JOYSTICK_DRIVE:
                    currentDrivetrainCommand = JoystickDrive(1.0);
                    break;
                case PRECISE:
                    currentDrivetrainCommand = JoystickDrive(PRECISE_SPEED_SCALE);
                    break;
                case LOCKED:
                    break;
                default:
                    m_state = DrivetrainStates.JOYSTICK_DRIVE;
            }
        }


        m_lastState = m_state;
        
        if (currentDrivetrainCommand != null) {
            currentDrivetrainCommand.schedule();
        }
            

    }

    // State Commands
    private final Command JoystickDrive(double speedMultiplier) {
        return new RunCommand(
                () -> {
                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_joystickTranslationX.getAsDouble() * speedMultiplier,
                            m_joystickTranslationY.getAsDouble() * speedMultiplier,
                            m_joystickRotationOmega.getAsDouble(),
                            getGyroYaw()));
                },
                this).ignoringDisable(true);//.andThen(stop());
    }

    // Helper Commands
    private final Command stop() {
        return new InstantCommand(
                () -> drive(new ChassisSpeeds(0d, 0d, 0d)),
                this);
    }

    public final static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

}
