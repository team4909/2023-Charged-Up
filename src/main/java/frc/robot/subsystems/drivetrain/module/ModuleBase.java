package frc.robot.subsystems.drivetrain.module;

public abstract class ModuleBase {

    public double drivePositionRad = 0d;
    public double driveVelocityRadPerSec = 0d;
    public double driveAppliedVolts = 0d;
    public double driveCurrentAmps = 0d;

    public double turnAbsolutePositionRad = 0d;
    public double turnPosition = 0d;
    public double turnVelocityRadPerSec = 0d;
    public double turnAppliedVolts = 0d;
    public double turnCurrentAmps = 0d;

    abstract void updateModuleInputs();

    abstract void setDrive(double input, double ff);

    abstract void setTurn(double input);

    // For physical modules only
    abstract void resetTurn();
}
