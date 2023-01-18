package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SimulatedModule extends AbstractModule {

    private FlywheelSim driveMotor;
    private FlywheelSim turnMotor;

    private double turnRelativePositionRad = 0d;
    private double turnAbsolutePositionRad = 0d;
    private double driveAppliedVolts = 0d;
    private double turnAppliedVolts = 0d;

    public SimulatedModule() {
        // Source for ratios: https://www.swervedrivespecialties.com/products/mk4i-swerve-module, using inertia values from 6328
        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0.025);
        turnMotor = new FlywheelSim(DCMotor.getFalcon500(1), 150d / 7d, 0.004);
    }

    @Override
    void updateModuleInputs() {
        double dt = Constants.PERIODIC_LOOP_DURATION;
        driveMotor.update(dt);
        turnMotor.update(dt);
        
        this.turnAbsolutePositionRad += turnMotor.getAngularVelocityRadPerSec() * dt;
        //Keep wheel in range (0, 2pi) //TODO should this be a while loop?
        if (turnAbsolutePositionRad < 0) turnAbsolutePositionRad += 2.0 * Math.PI;
        if (turnAbsolutePositionRad > 2.0 * Math.PI) turnAbsolutePositionRad -= 2.0 * Math.PI;

        super.drivePositionRad += driveMotor.getAngularVelocityRadPerSec() * dt;
    }
    
    @Override
    void setDriveVolts(double volts) {
        // TODO Auto-generated method stub
        
    }

    @Override
    void setTurnVolts(double volts) {
        // TODO Auto-generated method stub
        
    }


    
}
