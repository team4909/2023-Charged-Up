package frc.robot.subsystems.drivetrain.module;

import static frc.robot.Constants.DrivetrainConstants.FALCON_500_FREE_SPEED;
import static frc.robot.Constants.DrivetrainConstants.MODULE_CONFIGURATION;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public final class Module {

    private final ModuleBase m_module;
    private final int m_index;
    private final double m_wheelRadius = MODULE_CONFIGURATION.getWheelDiameter() / 2d;
    private final double MAX_VOLTAGE = 12d;
    private final double MAX_VELOCITY_METERS_PER_SECOND = FALCON_500_FREE_SPEED / 60.0 *
            MODULE_CONFIGURATION.getDriveReduction() *
            MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;

    private final PIDController m_turnMotorSimGains = new PIDController(23.0, 0.0, 0.1); // Tuned for SIM!
    private final PIDController m_turnMotorGains = new PIDController(0.2, 0.0, 0.0);
    private final PIDController m_turnPID;

    public Module(int index) {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule(index);
        m_index = index;
        m_turnPID = Constants.SIM ? m_turnMotorSimGains : m_turnMotorGains;
    }

    public void update() {
        m_module.updateModuleInputs();
        
        SmartDashboard.putNumber("Encoder " + m_index, m_module.turnAbsolutePositionRad);
        SmartDashboard.putString("State " + m_index, getModuleState().toString());
        SmartDashboard.putString("Pos " + m_index, getModulePosition().toString());
    }

    public void set(SwerveModuleState state) {
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getModuleAngle());
        m_module.setTurnVolts(m_turnPID.calculate(getModuleAngle().getRadians(), desiredState.angle.getRadians()));
        desiredState.speedMetersPerSecond *= Math.cos(m_turnPID.getPositionError()); // using cosine error:
                                                                                          // v_m = v_a * cos theta

        m_module.setDriveVolts(desiredState.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE);

    }

    private Rotation2d getModuleAngle() {
        return new Rotation2d(MathUtil.angleModulus(m_module.turnAbsolutePositionRad));
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

    public void stop() {
        m_module.setTurnVolts(0d);
        m_module.setDriveVolts(0d);
    }
}
