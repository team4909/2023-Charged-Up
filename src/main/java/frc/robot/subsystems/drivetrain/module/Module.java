package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final PIDController m_simTurnPID = new PIDController(23.0, 0.0, 0.1);
    private final SimpleMotorFeedforward m_driveFeedforward;

    public Module(int index) {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule(index);
        m_index = index;
        m_driveFeedforward = new SimpleMotorFeedforward(
                DrivetrainConstants.DRIVE_kS, DrivetrainConstants.DRIVE_kV, DrivetrainConstants.DRIVE_kA);
    }

    public void update() {
        m_module.updateModuleInputs();

        SmartDashboard.putString("Drivetrain/Module/State " + m_index, getModuleState().toString());
        SmartDashboard.putString("Drivetrain/Module/Position " + m_index, getModulePosition().toString());
        SmartDashboard.putNumber("Absolute Module Angle " + m_index, m_module.turnPosition);
        SmartDashboard.putNumber("Drivetrain/Actual Module Speed" + m_index,
                Math.abs(getModuleState().speedMetersPerSecond));
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
            var angleError = optimizedDesiredState.angle.getRadians()
                    - MathUtil.angleModulus(Math.toRadians(m_module.turnPosition));
            optimizedDesiredState.speedMetersPerSecond *= Math.cos(angleError);
            m_module.setTurn(convertDegreesToTicks(optimizedDesiredState.angle.getDegrees()));
            double speedTicks = convertMPStoTicks(optimizedDesiredState.speedMetersPerSecond);
            double ff = m_driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond);
            SmartDashboard.putNumber("Drivetrain/Drive FF", ff);
            SmartDashboard.putNumber("Drivetrain/Angle Error Rad", angleError);
            m_module.setDrive(speedTicks, ff);
        }
    }

    private Rotation2d getModuleAngle() {
        return Constants.SIM ? new Rotation2d(MathUtil.angleModulus(m_module.turnAbsolutePositionRad))
                : Rotation2d.fromDegrees(m_module.turnPosition);
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
        double motorRPM = RPM / DrivetrainConstants.DRIVE_REDUCTION;
        return motorRPM * (2048.0 / 600.0);
    }

    public static double convertTicksToRPM(double ticks) {
        double motorRPM = ticks * (600.0 / 2048.0);
        double mechRPM = motorRPM * DrivetrainConstants.DRIVE_REDUCTION;
        return mechRPM;
    }

    public void resetTurn() {
        if (m_module instanceof PhysicalModule) {
            m_module.resetTurn();
        }
    }
}
