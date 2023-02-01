package frc.robot.subsystems.drivetrain.module;

public abstract class ModuleBase {

    public double drivePositionRad = 0d;
    public double driveVelocityRadPerSec = 0d;
    public double driveAppliedVolts = 0d;
    public double driveCurrentAmps = 0d;

    public double turnAbsolutePosition = 0d;
    public double turnPositionRad = 0d;
    public double turnVelocityRadPerSec = 0d;
    public double turnAppliedVolts = 0d;
    public double turnCurrentAmps = 0d;

    abstract void updateModuleInputs();

    abstract void setDriveVolts(double volts);

    abstract void setTurn(double input);
}
