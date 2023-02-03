package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator {

    private SwerveDrivePoseEstimator m_swerveDrivePoseEstimator;

    public PoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        m_swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions,
                initialPoseMeters);

    }

}
