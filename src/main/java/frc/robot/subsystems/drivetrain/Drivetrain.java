package frc.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Module;

public class Drivetrain extends SubsystemBase {
    // private static Drivetrain instance = null;

    // double wheelBase = 31.625 * 0.0254;
    double wheelBase = 26.0 * 0.0254;
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(wheelBase, wheelBase);
    Translation2d m_frontRightLocation = new Translation2d(wheelBase, -wheelBase);
    Translation2d m_backLeftLocation = new Translation2d(-wheelBase, wheelBase);
    Translation2d m_backRightLocation = new Translation2d(-wheelBase, -wheelBase);
    Field2d field = new Field2d();

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation);

    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry;

    Pose2d m_pose;

    Joystick js0 = new Joystick(0);

    private final double FRONT_LEFT_ENC_OFFSET = 282.7;
    private final double FRONT_RIGHT_ENC_OFFSET = 186.2;
    private final double BACK_RIGHT_ENC_OFFSET = 91.2;
    private final double BACK_LEFT_ENC_OFFSET = 251.2; 

    Module leftModule = new Module("FrontLeft", 7, 8, 14, FRONT_LEFT_ENC_OFFSET);
    Module rightModule = new Module("FrontRight", 2, 1, 11, FRONT_RIGHT_ENC_OFFSET);
    Module backRightModule = new Module("BackRight", 4, 3, 12, BACK_RIGHT_ENC_OFFSET);
    Module backLeftModule = new Module("BackLeft", 6, 5, 13, BACK_LEFT_ENC_OFFSET);

    ShuffleboardTab drivetrainTab;

    SlewRateLimiter fwdBakRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter leftRightRateLimiter = new SlewRateLimiter(0.5);
    SlewRateLimiter turnRateLimiter = new SlewRateLimiter(0.5);

    private DoubleSupplier m_joystickTranslationX, m_joystickTranslationY, m_joystickRotationOmega;

    Pigeon2 pigeon = new Pigeon2(20);
    // Global variable for drive rate speed
    double g_driveRate;

    public Rotation2d getGyroHeading() {
        // // Get my gyro angle. We are negating the value because gyros return positive
        // // values as the robot turns clockwise. This is not standard convention that
        // is
        // // used by the WPILib classes.
        // var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public double getGyroPitch() {
        return pigeon.getPitch();
    }

    private Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    private static Drivetrain m_inst = null;

    public static Drivetrain getInstance() {
        if (m_inst == null) {
            m_inst = new Drivetrain();
        }
        return m_inst;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = leftModule.getPosition();
        positions[1] = rightModule.getPosition();
        positions[2] = backRightModule.getPosition();
        positions[3] = backLeftModule.getPosition();
        return positions;

    }

    private Drivetrain() {
        pigeon.setYaw(0);

        m_odometry = new SwerveDriveOdometry(
                m_kinematics,
                getGyroHeading(),
                getModulePositions());

        SmartDashboard.putBoolean("Done", false);
    }

    // private Drivetrain() {
    // drivetrainTab = Shuffleboard.getTab("drivetrain");

    // }

    // public static Drivetrain getInstance() {
    // if (instance == null) {
    // instance = new Drivetrain();
    // }
    // return instance;
    // }

    // public ShuffleboardTab getTab() {
    // return drivetrainTab;
    // }

    // this is alled every loop of the scheduler (~20ms)
    @Override
    public void periodic() {
        leftModule.periodic();
        rightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        SmartDashboard.putNumber("Pidgeon yaw", pigeon.getYaw());
        field.setRobotPose(m_odometry.getPoseMeters());
        SmartDashboard.putData(field);
        // Update the pose
        // button 8 on xbox is three lines button
        if (js0.getRawButton(8)) {
            leftModule.resetTurnEncoders();
            rightModule.resetTurnEncoders();
            backRightModule.resetTurnEncoders();
            backLeftModule.resetTurnEncoders();
        }
        // button 7 on xbox is two squares
        if (js0.getRawButton(7)) {
            pigeon.setYaw(0);
        }
        // Button 6 is right bumper/slow button
        // Speed is multiplied by 0.3 when held down
        if (js0.getRawButton(6)) {
            setDriveRate(0.3);
        } else {
            setDriveRate(1);
        }

        SmartDashboard.putString("odo", m_odometry.getPoseMeters().toString());
        SmartDashboard.putString("odo", m_odometry.getPoseMeters().toString());
        // SmartDashboard.putNumber("Pose roation", m_pose.getRotation().getDegrees()) ;
        // SmartDashboard.putNumber("Pose X", m_pose.getX());
        // SmartDashboard.putNumber("Pose Y", m_pose.getY());

        // DriveWithJoystick(js0);
        m_odometry.update(getGyroHeading(), getModulePositions());
    }

    public static double convertTicksToDegrees(double ticks) {
        // Cancoder: 2048 ticks per rotation
        // Steering gear ratio: 150/7:1
        double degrees = ticks * (1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0);
        return degrees;
    }

    public static double convertDegreesToTicks(double degrees) {

        double ticks = degrees * 1 / ((1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0));
        return ticks;
    }

    public void DriveWithJoystick(CommandXboxController js) {

        double leftRightDir = -1 * getDriveRate() * js.getRawAxis(0); // positive number means left
        double fwdBackDir = -1 * getDriveRate() * js.getRawAxis(1); // positive number means fwd
        double turn = -1 * getDriveRate() * js.getRawAxis(4); // positive number means clockwise

        // fwdBackDir = fwdBakRateLimiter.calculate(fwdBackDir);
        // leftRightDir = leftRightRateLimiter.calculate(leftRightDir);
        // turn = turnRateLimiter.calculate(turn);

        double deadband = .05;

        if (-deadband < leftRightDir && leftRightDir < deadband) {
            leftRightDir = 0;
        }

        if (-deadband < fwdBackDir && fwdBackDir < deadband) {
            fwdBackDir = 0;
        }

        if (-deadband < turn && turn < deadband) {
            turn = 0;
        }

        // Example chassis speeds: 1 meter per second forward, 3 meters
        // per second to the left, and rotation at 1.5 radians per second
        // counteclockwise.
        // ChassisSpeeds speeds = new ChassisSpeeds(fwdBackDir, leftRightDir, turn);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fwdBackDir * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                leftRightDir * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                turn * Drivetrain.MAX_OMEGA_RADIANS_PER_SECOND,
                Rotation2d.fromDegrees(pigeon.getYaw()));

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        setModuleStates(moduleStates);

    }

    public void drive(ChassisSpeeds speeds) {
        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        setModuleStates(moduleStates);
    }

    public void DriveWithVelocity(double fwdBackDir, double leftRightDir, double turn) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                fwdBackDir * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                leftRightDir * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                turn * Drivetrain.MAX_OMEGA_RADIANS_PER_SECOND,
                Rotation2d.fromDegrees(pigeon.getYaw()));

        // Convert to module states
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        setModuleStates(moduleStates);
    }

    public void setJoystickSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_joystickTranslationX = x;
        m_joystickTranslationY = y;
        m_joystickRotationOmega = omega;
    }

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4;
    public static final double MAX_OMEGA_RADIANS_PER_SECOND = 2.5;
    private static final double MAX_ANGULAR_SPEED = 4; // rads per sec

    public void setModuleStates(SwerveModuleState[] moduleStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backRight = moduleStates[3];
        SwerveModuleState backLeft = moduleStates[2];

        leftModule.setModuleState(frontLeft);
        rightModule.setModuleState(frontRight);
        backRightModule.setModuleState(backRight);
        backLeftModule.setModuleState(backLeft);
    }

    public Command resetOdometry() {
        return new InstantCommand(
                () -> this.m_odometry.resetPosition(getGyroHeading(), getModulePositions(), getPose()));
    }

    public Command runPath(PathPlannerTrajectory traj) {
        PIDController xyController = new PIDController(5, 0, 0);

        return new PPSwerveControllerCommand(
                traj,
                this::getPose,
                this.m_kinematics,
                xyController,
                xyController,
                new PIDController(5, 0, 0),
                this::setModuleStates,
                this);
    }

    public Command traj(PathPlannerTrajectory traj, boolean isFirstPath) {

        PIDController xyController = new PIDController(5, 0, 0);

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath)
                        this.m_odometry.resetPosition(getGyroHeading(), getModulePositions(),
                                traj.getInitialHolonomicPose());

                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose,
                        this.m_kinematics,
                        xyController,
                        xyController,
                        new PIDController(5, 0, 0),
                        this::setModuleStates,
                        this),
                new InstantCommand(() -> {
                    SmartDashboard.putBoolean("Done", true);
                })).andThen(new InstantCommand(() -> {
                    setModuleStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
                }));
    }

    public void setDriveRate(double driveRate) {
        g_driveRate = driveRate;
    }

    public double getDriveRate() {
        return g_driveRate;
    }

    public void setGyro(double d) {
        pigeon.setYaw(d);
    }

    public Command SnapToAngle(double angle) {
        PIDController snapPID = new PIDController(0.012, 0.0, 0.0);
        snapPID.enableContinuousInput(-180, 180);
        snapPID.setSetpoint(angle);
        snapPID.setTolerance(2);
        return new RunCommand(
                () -> {
                    double omega = snapPID.calculate(
                            (MathUtil.inputModulus(getGyroHeading().getDegrees(), -180, 180)),
                            snapPID.getSetpoint());
                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            -m_joystickTranslationX.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                            -m_joystickTranslationY.getAsDouble() * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                            (m_joystickRotationOmega.getAsDouble() + omega) * MAX_ANGULAR_SPEED, getGyroHeading()));
                },
                this).finallyDo((i) -> snapPID.close()).until(() -> snapPID.atSetpoint());
    }
}