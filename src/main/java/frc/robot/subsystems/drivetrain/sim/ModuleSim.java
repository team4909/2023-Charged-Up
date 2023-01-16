package frc.robot.subsystems.drivetrain.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ModuleSim {

    private FlywheelSim driveMotor;
    private FlywheelSim turnMotor;

    public ModuleSim() {
        // Source for ratios: https://www.swervedrivespecialties.com/products/mk4i-swerve-module
        driveMotor = new FlywheelSim(DCMotor.getFalcon500(1), 6.75, 0);
        turnMotor = new FlywheelSim(DCMotor.getFalcon500(1), 150d / 7d, 0);
    }
    
    public static class ModuleSimInputs {
        //copied from 6328
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveVelocityFilteredRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
        public double[] driveTempCelcius = new double[] {};
        
        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
    }
}