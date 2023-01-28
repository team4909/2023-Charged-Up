package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;
import static frc.robot.Constants.DrivetrainConstants.PRECISE_SPEED_SCALE;
import static frc.robot.Constants.DrivetrainConstants.MAX_DRIVETRAIN_SPEED;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.module.Module;

public class Drivetrain extends SubsystemBase {

    // #region Fields
    private static Drivetrain m_instance = null;

    private final Module[] m_modules = new Module[4]; // 0-FL 1-FR 2-BL 3-BR
    private final Field2d m_field = new Field2d();

    // TODO these should be the same and should be an IDLE state
    private DrivetrainStates m_state = DrivetrainStates.JOYSTICK_DRIVE;
    private DrivetrainStates m_lastState = null;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private Rotation2d m_simChassisAngle = new Rotation2d();
    private DoubleSupplier m_joystickTranslationX, m_joystickTranslationY, m_joystickRotationOmega;
    private SwerveDriveOdometry m_odometry;
    private Pose2d m_pose;

    private final Pigeon2 m_pigeon = new Pigeon2(PIGEON_ID);
    private final Translation2d[] m_moduleTranslations = new Translation2d[] {
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // FL
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // FR
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // BL
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // BR
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_moduleTranslations);
    private final double MAX_ANGULAR_SPEED = MAX_DRIVETRAIN_SPEED / Arrays.stream(m_moduleTranslations)
            .map(t -> t.getNorm())
            .max(Double::compare).get();
    // #endregion

    public Consumer<SwerveModuleState[]> m_swerveModuleConsumer = (states) -> drive(m_kinematics.toChassisSpeeds(states));
    public Supplier<Pose2d> m_poseSupplier = () -> m_pose;

    public enum DrivetrainStates {
        JOYSTICK_DRIVE("Joystick Drive"),
        AUTONOMOUS("Autonomous"),
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
        m_pigeon.clearStickyFaults();
        for (int i = 0; i < m_modules.length; i++)
            m_modules[i] = new Module(i);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroYaw(), getSwerveModulePositions());
        m_pose = m_odometry.getPoseMeters();
        SmartDashboard.putData(m_field);
    }

    @Override
    public void periodic() {
        stateMachine();
        m_field.setRobotPose(m_pose);

        double dt = Constants.PERIODIC_LOOP_DURATION;

        Twist2d dModuleState = new Pose2d()
                .log(new Pose2d(
                        m_chassisSpeeds.vxMetersPerSecond * dt,
                        m_chassisSpeeds.vyMetersPerSecond * dt,
                        new Rotation2d(
                                m_chassisSpeeds.omegaRadiansPerSecond * dt)));
        m_simChassisAngle = m_simChassisAngle.plus(new Rotation2d(dModuleState.dtheta));
        ChassisSpeeds adjustedSpeeds = new ChassisSpeeds(
                dModuleState.dx / dt,
                dModuleState.dy / dt,
                dModuleState.dtheta / dt);
        SwerveModuleState[] setpointModuleStates = m_kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointModuleStates, MAX_DRIVETRAIN_SPEED);

        for (int i = 0; i < 4; i++) {
            m_modules[i].update();
            m_modules[i].set(setpointModuleStates[i]);
        }

        m_pose = m_odometry.update(getGyroYaw(), getSwerveModulePositions());
        telem();
    }

    public void telem() {
        SmartDashboard.putString("DrivetrainState", m_state.toString());
        SmartDashboard.putString("chassis speeds", m_chassisSpeeds.toString());
        SmartDashboard.putString("Odo", m_odometry.getPoseMeters().toString());
        SmartDashboard.putString("sim angle", m_simChassisAngle.toString());
    }
    
    public void setFieldTrajectory(Trajectory t) {
        m_field.getObject("traj").setTrajectory(t);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    private Rotation2d getGyroYaw() {
        return Constants.SIM ? m_simChassisAngle : Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < m_modules.length; i++)
            positions[i] = m_modules[i].getModulePosition();
        return positions;
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
                    currentDrivetrainCommand = JoystickDrive(1d, 1d);
                    break;
                case AUTONOMOUS:
                    break;
                case PRECISE:
                    currentDrivetrainCommand = JoystickDrive(PRECISE_SPEED_SCALE, PRECISE_SPEED_SCALE);
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

    public void setState(DrivetrainStates state) {
        m_state = state;
    }

    // #region State Commands
    private Command JoystickDrive(double linearSpeedScale, double angularSpeedScale) {
        return new RunCommand(
                () -> {
                    final double x = -m_joystickTranslationX.getAsDouble();
                    final double y = -m_joystickTranslationY.getAsDouble();
                    final Rotation2d theta = new Rotation2d(x, y);
                    double magnitude = Math.hypot(x, y);
                    double omega = -m_joystickRotationOmega.getAsDouble();

                    magnitude = deadband(magnitude, 0.05);
                    omega = deadband(omega, 0.05);
                    magnitude = squareAxis(magnitude);
                    omega = squareAxis(omega);

                    magnitude *= linearSpeedScale;
                    omega *= angularSpeedScale;

                    Translation2d linearVelocity = new Pose2d(new Translation2d(), theta)
                            .transformBy(new Transform2d(new Translation2d(magnitude, 0d), new Rotation2d()))
                            .getTranslation();

                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.getX() * MAX_DRIVETRAIN_SPEED,
                            linearVelocity.getY() * MAX_DRIVETRAIN_SPEED,
                            omega * MAX_ANGULAR_SPEED,
                            getGyroYaw()));
                },
                this).andThen(stop()).ignoringDisable(true);
    }

    private Command stop() {
        return new InstantCommand(
                () -> drive(new ChassisSpeeds()),
                this);
    }
    // #endregion
    
    public Command zeroGyro() {
        return new InstantCommand(() -> m_pigeon.setYaw(0d));
    }

    private double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0d)
                return (value - deadband) / (1d - deadband);
            else
                return (value + deadband) / (1d - deadband);
        } else {
            return 0d;
        }
    }

    private double squareAxis(double value) {
        return Math.copySign(value * value, value);
    }

    public final static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

}
