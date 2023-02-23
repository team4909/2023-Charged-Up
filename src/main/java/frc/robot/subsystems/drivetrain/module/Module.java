package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

    private final PIDController m_simTurnPID = new PIDController(23.0, 0.0, 0.1); // Tuned for SIM!

    public Module(int index) {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule(index);
        m_index = index;
    }

    public void update() {
        m_module.updateModuleInputs();

        SmartDashboard.putNumber("Encoder " + m_index, m_module.turnAbsolutePosition);
        SmartDashboard.putString("State " + m_index, getModuleState().toString());
        SmartDashboard.putString("Pos " + m_index, getModulePosition().toString());
    }

    public void set(SwerveModuleState state) {
        SwerveModuleState desiredState;
        if (Constants.SIM) {
            desiredState = SwerveModuleState.optimize(state, getModuleAngle());
            m_module.setTurn(m_simTurnPID.calculate(getModuleAngle().getRadians(), desiredState.angle.getRadians()));
        } else {
            desiredState = SwerveModuleState.optimize(state, getModuleAngle());
            // desiredState = CTREModuleState.optimize(state, getModuleAngle());
            m_module.setTurn(convertDegreesToTicks(desiredState.angle.getDegrees()));
        }
        m_module.setDriveVolts(desiredState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);
    }

    private Rotation2d getModuleAngle() {
        return Constants.SIM ? new Rotation2d(MathUtil.angleModulus(m_module.turnAbsolutePosition))
                : Rotation2d.fromDegrees(m_module.turnAbsolutePosition);
    }

    private double getPositionMeters() {
        return m_module.drivePositionRad * m_wheelRadius;
    }

    private double getVelocityMetersPerSec() {
        return m_module.driveVelocityRadPerSec * m_wheelRadius;
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
}
