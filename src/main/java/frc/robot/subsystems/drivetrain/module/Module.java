package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Module {

    private ModuleBase m_module;
    private double m_wheelRadius = Constants.DrivetrainConstants.MODULE_CONFIGURATION.getWheelDiameter() / 2d; 

    private final PIDController m_turnMotorPID = new PIDController(0.2, 0.0, 0.1); // This is just used as a container

    public Module() {
        m_module = Constants.SIM ? new SimulatedModule() : new PhysicalModule();
    }

    void update() {
        m_module.updateModuleInputs();
        
    }

    public double getPositionMeters() {
        return m_module.drivePositionRad * m_wheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return m_module.driveVelocityRadPerSec * m_wheelRadius;
    }

    public void stop() {
        m_module.setTurnVolts(0d);
        m_module.setDriveVolts(0d);
    }
}
