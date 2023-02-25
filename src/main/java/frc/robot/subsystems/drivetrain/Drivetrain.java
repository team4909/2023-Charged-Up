package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.module.Module;

public class Drivetrain extends SubsystemBase {

    // #region Fields
    private static Drivetrain m_instance = null;

    private final Module[] m_modules = new Module[4]; // 0-FL 1-FR 2-BL 3-BR
    private final Field2d m_field = new Field2d();

    private DrivetrainStates m_state = DrivetrainStates.IDLE;
    private DrivetrainStates m_lastState;
    private HashMap<String, ?> m_stateArgs;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private Rotation2d m_simChassisAngle = new Rotation2d();
    private DoubleSupplier m_joystickTranslationX, m_joystickTranslationY, m_joystickRotationOmega;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Pose2d m_pose;

    private final Pigeon2 m_pigeon = new Pigeon2(DrivetrainConstants.PIGEON_ID);
    private final Translation2d[] m_moduleTranslations = new Translation2d[] {
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // FL
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // FR
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // BL
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // BR
    };
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_moduleTranslations);
    private final double MAX_ANGULAR_SPEED = 4;
    // #endregion

    public Consumer<SwerveModuleState[]> m_swerveModuleConsumer = (states) -> drive(
            m_kinematics.toChassisSpeeds(states));
    public Supplier<Pose2d> m_poseSupplier = () -> m_pose;

    public enum DrivetrainStates {
        IDLE("Idle"),
        JOYSTICK_DRIVE("Joystick Drive"),
        TRAJECTORY_DRIVE("Trajectory Drive"),
        PRECISE("Precise"),
        SNAP_TO_ANGLE("Snapping To Angle");

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
        m_pose = new Pose2d();
        // Using default std deviations values
        m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroYaw(), getSwerveModulePositions(), m_pose,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.9, 0.9, 0.9));
        SmartDashboard.putData(m_field);
    }

    @Override
    public void periodic() {
        stateMachine();
        m_field.setRobotPose(m_pose);

        final double dt = Constants.PERIODIC_LOOP_DURATION;

        Twist2d dModuleState = new Pose2d()
                .log(new Pose2d(
                        m_chassisSpeeds.vxMetersPerSecond * dt,
                        m_chassisSpeeds.vyMetersPerSecond * dt,
                        new Rotation2d(
                                m_chassisSpeeds.omegaRadiansPerSecond * dt)));
        m_simChassisAngle = m_simChassisAngle.plus(new Rotation2d(dModuleState.dtheta));
        SwerveModuleState[] setpointModuleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointModuleStates, DrivetrainConstants.MAX_DRIVETRAIN_SPEED);

        for (int i = 0; i < 4; i++) {
            m_modules[i].update();
            m_modules[i].set(setpointModuleStates[i]);
        }

        m_pose = m_poseEstimator.update(getGyroYaw(), getSwerveModulePositions());
        telem();
    }

    public void telem() {
        SmartDashboard.putString("DrivetrainState", m_state.toString());
        SmartDashboard.putBoolean("Joystick", isJoystickInputPresent());
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

    private boolean isJoystickInputPresent() {
        return !Stream
                .of(m_joystickTranslationX.getAsDouble(), m_joystickTranslationY.getAsDouble(),
                        m_joystickRotationOmega.getAsDouble())
                .filter((input) -> deadband(input, DrivetrainConstants.DEADBAND) != 0)
                .collect(Collectors.toList()).isEmpty();
    }

    private void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    private void stateMachine() {
        Command currentDrivetrainCommand = null;
        if (!m_state.equals(m_lastState)) {
            switch (m_state) {
                case IDLE:
                    currentDrivetrainCommand = StopIdle().repeatedly().until(this::isJoystickInputPresent)
                            .finallyDo((interrupted) -> {
                                if (!interrupted)
                                    setState(DrivetrainStates.JOYSTICK_DRIVE).schedule();
                            });
                    break;
                case JOYSTICK_DRIVE:
                    currentDrivetrainCommand = JoystickDrive(1d, 1d, DrivetrainConstants.DEADBAND)
                            .until(() -> !isJoystickInputPresent())
                            .finallyDo((interrupted) -> {
                                if (!interrupted)
                                    setState(DrivetrainStates.IDLE).schedule();
                            });
                    break;
                case TRAJECTORY_DRIVE:
                    currentDrivetrainCommand = TrajectoryDrive(
                            (PathPlannerTrajectory) m_stateArgs.get("Trajectory"),
                            (double) m_stateArgs.get("Timeout")).andThen(setState(DrivetrainStates.IDLE));
                    break;
                case PRECISE:
                    currentDrivetrainCommand = JoystickDrive(DrivetrainConstants.PRECISE_SPEED_SCALE,
                            DrivetrainConstants.PRECISE_SPEED_SCALE, DrivetrainConstants.DEADBAND / 2d);
                    break;
                case SNAP_TO_ANGLE:
                    currentDrivetrainCommand = SnapToAngle((double) m_stateArgs.get("Angle"));
                    break;
                default:
                    m_state = DrivetrainStates.IDLE;
            }
        }

        m_lastState = m_state;

        if (currentDrivetrainCommand != null) {
            currentDrivetrainCommand.schedule();
        }
    }

    public Command setState(DrivetrainStates state) {
        return new InstantCommand(() -> m_state = state);
    }

    public Command setState(DrivetrainStates state, HashMap<String, ?> stateArgs) {
        return new InstantCommand(() -> {
            m_state = state;
            m_stateArgs = stateArgs;
        });
    }

    // #region State Commands
    private Command JoystickDrive(double linearSpeedScale, double angularSpeedScale, double deadband) {
        return new RunCommand(
                () -> {
                    final double x = -m_joystickTranslationX.getAsDouble();
                    final double y = -m_joystickTranslationY.getAsDouble();
                    final Rotation2d theta = new Rotation2d(x, y);
                    double magnitude = Math.hypot(x, y);
                    double omega = -m_joystickRotationOmega.getAsDouble();

                    magnitude = deadband(magnitude, deadband);
                    omega = deadband(omega, deadband);
                    magnitude = cubeAxis(magnitude);
                    omega = cubeAxis(omega);

                    magnitude *= linearSpeedScale;
                    omega *= angularSpeedScale;

                    Translation2d linearVelocity = new Pose2d(new Translation2d(), theta)
                            .transformBy(new Transform2d(new Translation2d(magnitude, 0d), new Rotation2d()))
                            .getTranslation();

                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.getX() * DrivetrainConstants.MAX_DRIVETRAIN_SPEED,
                            linearVelocity.getY() * DrivetrainConstants.MAX_DRIVETRAIN_SPEED,
                            omega * MAX_ANGULAR_SPEED,
                            getGyroYaw()));
                },
                this);

    }

    private Command StopIdle() {
        return new InstantCommand(
                () -> drive(new ChassisSpeeds()), this);
    }

    private Command TrajectoryDrive(PathPlannerTrajectory trajectory, double timeout) {
        return new InstantCommand(() -> setFieldTrajectory(trajectory)).andThen(
                new PPSwerveControllerCommand(
                        trajectory,
                        m_poseSupplier,
                        getKinematics(),
                        new PIDController(10, 0, 0),
                        new PIDController(10, 0, 0),
                        new PIDController(5, 0, 0),
                        m_swerveModuleConsumer,
                        false,
                        this)
                        .withTimeout(timeout));
    }

    private Command SnapToAngle(double angle) {
        PIDController snapPID = new PIDController(0.012, 0.0, 0.0);
        snapPID.enableContinuousInput(-180, 180);
        snapPID.setSetpoint(angle);
        return new RunCommand(
                () -> {
                    double omega = snapPID.calculate(
                            (MathUtil.inputModulus(getGyroYaw().getDegrees(), -180, 180)),
                            snapPID.getSetpoint());
                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            // TODO This needs to deal with deadband
                            -m_joystickTranslationX.getAsDouble() * DrivetrainConstants.MAX_DRIVETRAIN_SPEED,
                            -m_joystickTranslationY.getAsDouble() * DrivetrainConstants.MAX_DRIVETRAIN_SPEED,
                            (m_joystickRotationOmega.getAsDouble() + omega) * MAX_ANGULAR_SPEED, getGyroYaw()));
                },
                this).andThen(() -> snapPID.close());
    }
    // #endregion

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getSwerveModulePositions(), pose);
    }

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

    private double cubeAxis(double value) {
        return Math.copySign(Math.pow(value, 3), value);
    }

    public final static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

}
