package frc.lib.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getStateDistance();
}
