package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    
    private static Drivetrain m_instance = null;

    private enum DrivetrainStates {
        IDLE("Idle"),
        DRIVING("Driving"),
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

    public static Drivetrain getInstance() {
        if (m_instance == null) {
            m_instance = new Drivetrain();
        }
        return m_instance;
    }

}
