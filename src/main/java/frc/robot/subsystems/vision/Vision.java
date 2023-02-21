package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {

    private static Vision m_instance = null;
    private LimelightResults m_visionResults;

    private Vision() {

    }

    @Override
    public void periodic() {
        m_visionResults = LimelightHelpers.getLatestResults("limelight");
        SmartDashboard.putString("Vision Pose", this.generatePose().toString());
    }

    public static Vision getInstance() {
        if (m_instance == null) {
            m_instance = new Vision();
        }
        return m_instance;
    }

    private Pose3d generatePose() {
        double[] poseCoords = m_visionResults.targetingResults.botpose;
        return new Pose3d(
                new Translation3d(poseCoords[0], poseCoords[1], poseCoords[2]),
                new Rotation3d(poseCoords[3], poseCoords[4], poseCoords[5]));
    }

}
