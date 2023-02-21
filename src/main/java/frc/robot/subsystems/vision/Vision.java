package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static Vision m_instance = null;

    private Vision() {

    }

    @Override
    public void periodic() {
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

}
