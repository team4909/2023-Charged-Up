package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain m_instance = null;
    private DrivetrainStates m_state;
    private DrivetrainStates m_lastState;

    private Pigeon2 m_pigeon;

    private ChassisSpeeds m_chassisSpeeds;
    private DoubleSupplier m_joystickTranslationX;
    private DoubleSupplier m_joystickTranslationY;
    private DoubleSupplier m_joystickRotationOmega;

    private enum DrivetrainStates {
        JOYSTICK_DRIVE("Joystick Drive"),
        PRECISE("Precise"),
        LOCKED("Locked");

        String stateName;

        private DrivetrainStates(String name) {
            this.stateName = name;
        }

        public String toString() {
            return this.stateName;
        }
    }

    private Drivetrain() {

    }

    public void periodic() {
        stateMachine();
    }

    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }


    public void setJoystickSuppliers(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        m_joystickTranslationX = x;
        m_joystickTranslationY = y;
        m_joystickRotationOmega = omega;
    }

    private void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    private void stateMachine() {
        Command currentDrivetrainCommand = null;
        if (m_state != m_lastState) {
            switch (m_state) {
                case JOYSTICK_DRIVE:
                    currentDrivetrainCommand = joystickDrive(1.0);
                    break;
                case PRECISE:
                    currentDrivetrainCommand = joystickDrive(0.2);
                    break;
                case LOCKED:
                    break;
                default:
                    m_state = DrivetrainStates.JOYSTICK_DRIVE;
            }
        }

        if (currentDrivetrainCommand != null)
            currentDrivetrainCommand.schedule();

    }

    // State Commands
    private final Command joystickDrive(double speedMultiplier) {
        return new RunCommand(
                () -> {
                    drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_joystickTranslationX.getAsDouble() * speedMultiplier,
                            m_joystickTranslationY.getAsDouble() * speedMultiplier,
                            m_joystickRotationOmega.getAsDouble(),
                            getGyroYaw())); // @TODO add gyroscope getter
                },
                this).andThen(stop());
    }

    // Helper Commands
    private final Command stop() {
        return new InstantCommand(
                () -> drive(new ChassisSpeeds(0d, 0d, 0d)),
                this);
    }

    public final static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

}
