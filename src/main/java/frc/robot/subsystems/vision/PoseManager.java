package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseManager {

    private SwerveDrivePoseEstimator m_swerveDrivePoseEstimator;
    private Vision m_vision = Vision.getInstance();

    public PoseManager(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {

        m_swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public Pose2d updatePose(Rotation2d currentGyroAngle, SwerveModulePosition[] modulePositions) {
        m_swerveDrivePoseEstimator.update(currentGyroAngle, modulePositions);
        Optional<EstimatedRobotPose> result = m_vision
                .getEstimatedGlobalPose(m_swerveDrivePoseEstimator.getEstimatedPosition());

        if (result.isPresent()) {
            EstimatedRobotPose cameraPose = result.get();
            m_swerveDrivePoseEstimator.addVisionMeasurement(
                    cameraPose.estimatedPose.toPose2d(),
                    cameraPose.timestampSeconds);
        }

        return m_swerveDrivePoseEstimator.getEstimatedPosition();
    }

}
