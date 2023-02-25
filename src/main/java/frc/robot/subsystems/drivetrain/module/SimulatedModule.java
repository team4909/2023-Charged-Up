package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public final class SimulatedModule extends ModuleBase {

    private final FlywheelSim driveMotor;
    private final FlywheelSim turnMotor;

    private double m_turnRelativePositionRad = 0d;
    private double m_turnAbsolutePositionRad = 0d;
    private double m_driveAppliedVolts = 0d;
    private double m_turnAppliedVolts = 0d;

    public SimulatedModule() {
        // Ratios: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 8e-10);
        turnMotor = new FlywheelSim(DCMotor.getFalcon500(1), 150d / 7d, 4e-11);
    }

    @Override
    void updateModuleInputs() {
        final double dt = Constants.PERIODIC_LOOP_DURATION;

        driveMotor.update(dt);
        turnMotor.update(dt);

        m_turnAbsolutePositionRad += turnMotor.getAngularVelocityRadPerSec() * dt;
        // Keep wheel in range (0, 2pi)
        if (m_turnAbsolutePositionRad < 0)
            m_turnAbsolutePositionRad += 2.0 * Math.PI;
        if (m_turnAbsolutePositionRad > 2.0 * Math.PI)
            m_turnAbsolutePositionRad -= 2.0 * Math.PI;

        super.driveAppliedVolts = m_driveAppliedVolts;
        super.drivePositionRad += driveMotor.getAngularVelocityRadPerSec() * dt;
        super.driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
        super.driveCurrentAmps = Math.abs(driveMotor.getCurrentDrawAmps());

        super.turnAbsolutePosition = m_turnAbsolutePositionRad;
        super.turnPositionRad = m_turnRelativePositionRad;
        super.turnAppliedVolts = m_turnAppliedVolts;
        super.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
        super.turnCurrentAmps = Math.abs(turnMotor.getCurrentDrawAmps());
    }

    @Override
    void setDriveVolts(double volts) {
        m_driveAppliedVolts = MathUtil.clamp(volts, -12d, 12d);
        driveMotor.setInputVoltage(m_driveAppliedVolts);
    }

    @Override
    void setTurn(double volts) {
        m_turnAppliedVolts = MathUtil.clamp(volts, -12d, 12d);
        turnMotor.setInputVoltage(m_turnAppliedVolts);
    }
}
