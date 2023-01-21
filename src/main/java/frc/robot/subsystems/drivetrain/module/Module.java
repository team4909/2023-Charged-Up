package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class Module {

    private ModuleBase m_module;
    private double m_wheelRadius = Constants.DrivetrainConstants.MODULE_CONFIGURATION.getWheelDiameter() / 2d; 
    private double m_driveReduction = Constants.DrivetrainConstants.MODULE_CONFIGURATION.getDriveReduction();

    private final PIDController m_turnMotorPID = new PIDController(0.2, 0.0, 0.1); // This is just used as a container

    public Module() {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule();
    }

    void update() {
        m_module.updateModuleInputs();
    }

    void set(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        final double MAX_VOLTAGE = 12d; 
        final double MAX_VELOCITY_METERS_PER_SECOND = Constants.DrivetrainConstants.FALCON_500_FREE_SPEED / 60.0 *
                m_driveReduction *
                m_wheelRadius * 2d * Math.PI;

        var driveVolts = state.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        
        
        m_module.setDriveVolts(driveVolts);
        m_module.setTurnVolts(0);
        
    }

    public Rotation2d getAngle() {
        return new Rotation2d(MathUtil.angleModulus(m_module.turnAbsolutePositionRad));
    }

    public double getPositionMeters() {
        return m_module.drivePositionRad * m_wheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return m_module.driveVelocityRadPerSec * m_wheelRadius;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public void stop() {
        m_module.setTurnVolts(0d);
        m_module.setDriveVolts(0d);
    }
}
