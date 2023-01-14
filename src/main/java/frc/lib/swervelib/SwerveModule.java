package frc.lib.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    double getDriveVelocity();

    double getDriveDistance();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    default SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getSteerAngle()));
    }

    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getSteerAngle()));
    }
}
