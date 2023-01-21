package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SimulatedModule extends ModuleBase {

    private FlywheelSim driveMotor;
    private FlywheelSim turnMotor;

    private double m_turnRelativePositionRad = 0d;
    private double m_turnAbsolutePositionRad = 0d;
    private double m_driveAppliedVolts = 0d;
    private double m_turnAppliedVolts = 0d;

    public SimulatedModule() {
        // Source for ratios:
        // https://www.swervedrivespecialties.com/products/mk4i-swerve-module, using
        // inertia values from 6328
        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
        turnMotor = new FlywheelSim(DCMotor.getFalcon500(1), 150d / 7d, 0.004);
    }

    @Override
    void updateModuleInputs() {
        final double dt = Constants.PERIODIC_LOOP_DURATION;

        driveMotor.update(dt);
        turnMotor.update(dt);

        m_turnAbsolutePositionRad += turnMotor.getAngularVelocityRadPerSec() * dt;
        // Keep wheel in range (0, 2pi) //TODO should this be a while loop?
        if (m_turnAbsolutePositionRad < 0)
            m_turnAbsolutePositionRad += 2.0 * Math.PI;
        if (m_turnAbsolutePositionRad > 2.0 * Math.PI)
            m_turnAbsolutePositionRad -= 2.0 * Math.PI;

        super.driveAppliedVolts = m_driveAppliedVolts;
        super.drivePositionRad += driveMotor.getAngularVelocityRadPerSec() * dt;
        super.driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
        super.driveCurrentAmps = new double[] { Math.abs(driveMotor.getCurrentDrawAmps()) };

        super.turnAbsolutePositionRad = m_turnAbsolutePositionRad;
        super.turnPositionRad = m_turnRelativePositionRad;
        super.turnAppliedVolts = m_turnAppliedVolts;
        super.turnVelocityRadPerSec = turnMotor.getAngularVelocityRadPerSec();
        super.turnCurrentAmps = new double[] { Math.abs(turnMotor.getCurrentDrawAmps()) };
    }

    @Override
    void setDriveVolts(double volts) {
        m_driveAppliedVolts = volts;
        driveMotor.setInputVoltage(m_driveAppliedVolts);
    }

    @Override
    void setTurnVolts(double volts) {
        m_turnAppliedVolts = volts;
        turnMotor.setInputVoltage(m_turnAppliedVolts);
    }
}
