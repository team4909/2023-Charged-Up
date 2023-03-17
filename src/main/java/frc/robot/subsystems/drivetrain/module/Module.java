package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.team364.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public final class Module {

    private final ModuleBase m_module;
    private final int m_index;
    private final double m_wheelRadius = DrivetrainConstants.WHEEL_DIAMETER / 2d;
    private final double MAX_VOLTAGE = 12d;
    private final double MAX_VELOCITY_METERS_PER_SECOND = DrivetrainConstants.FALCON_500_FREE_SPEED / 60.0 *
            DrivetrainConstants.DRIVE_REDUCTION *
            DrivetrainConstants.WHEEL_DIAMETER * Math.PI;
    private final PIDController m_simTurnPID = new PIDController(23.0, 0.0, 0.1); // Tuned for SIM!
    private double m_lastAngle;
    private final SimpleMotorFeedforward m_driveFeedforward;

    public Module(int index) {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule(index);
        m_index = index;
        m_lastAngle = getModuleState().angle.getDegrees();
        m_driveFeedforward = new SimpleMotorFeedforward(0.24634, 0.68591, 0.18461);
    }

    public void update() {
        m_module.updateModuleInputs();
        // SmartDashboard.putString("Drivetrain/Module/State " + m_index,
        // getModuleState().toString());
        // SmartDashboard.putString("Drivetrain/Module/Position " + m_index,
        // getModulePosition().toString());
    }

    public void set(SwerveModuleState desiredstate) {
        SwerveModuleState optimizedDesiredState;
        if (Constants.SIM) {
            optimizedDesiredState = SwerveModuleState.optimize(desiredstate, getModuleAngle());
            m_module.setTurn(
                    m_simTurnPID.calculate(getModuleAngle().getRadians(), optimizedDesiredState.angle.getRadians()));
            m_module.setDrive(
                    optimizedDesiredState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 0d);
        } else {
            optimizedDesiredState = CTREModuleState.optimize(desiredstate, getModuleState().angle);
            m_module.setTurn(convertDegreesToTicks(optimizedDesiredState.angle.getDegrees()));
            double speedTicks = convertMPStoTicks(optimizedDesiredState.speedMetersPerSecond);
            m_module.setDrive(speedTicks, m_driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond));
        }

        m_lastAngle = optimizedDesiredState.angle.getDegrees();
    }

    private Rotation2d getModuleAngle() {
        return Constants.SIM ? new Rotation2d(MathUtil.angleModulus(m_module.turnAbsolutePosition))
                : Rotation2d.fromDegrees(m_module.turnPositionRad);
    }

    private double getPositionMeters() {
        return m_module.drivePositionRad * m_wheelRadius;
    }

    private double getVelocityMetersPerSec() {
        return m_module.driveVelocityRadPerSec;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPositionMeters(), getModuleAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getModuleAngle());
    }

    // Util
    public static double convertTicksToDegrees(double ticks) {
        double degrees = ticks * (1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0);
        return degrees;
    }

    public static double convertDegreesToTicks(double degrees) {
        double ticks = degrees * 1 / ((1.0 / 2048.0) * (1.0 / (150 / 7)) * (360.0 / 1.0));
        return ticks;
    }

    public static double convertMPStoTicks(double mps) {
        double wheelRPM = ((mps * 60) / DrivetrainConstants.WHEEL_CIRCUMFERENCE);
        return convertRPMToTicks(wheelRPM);
    }

    public static double convertTicksToMPS(double ticks) {
        double wheelRPM = convertTicksToRPM(ticks);
        double wheelMPS = (wheelRPM * DrivetrainConstants.WHEEL_CIRCUMFERENCE) / 60;
        return wheelMPS;
    }

    public static double convertRPMToTicks(double RPM) {
        double motorRPM = RPM * 6.75; // TODO extract into a const
        return motorRPM * (2048.0 / 600.0);
    }

    public static double convertTicksToRPM(double ticks) {
        double motorRPM = ticks * (600.0 / 2048.0);
        double mechRPM = motorRPM / 6.75;
        return mechRPM;
    }

    public void resetTurn() {
        if (m_module instanceof PhysicalModule) {
            m_module.resetTurn();
        }
    }
}
