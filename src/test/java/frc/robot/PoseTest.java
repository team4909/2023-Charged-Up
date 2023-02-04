package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.vision.PoseManager;

@TestInstance(Lifecycle.PER_CLASS)
public class PoseTest {

    private PoseManager m_poseManager;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(26 / 2.0, 26 / 2.0),
            new Translation2d(26 / 2.0, -26 / 2.0),
            new Translation2d(-26 / 2.0, 26 / 2.0),
            new Translation2d(-26 / 2.0, -26 / 2.0));

    private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
    };

    @BeforeAll
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void tag1() {
        Rotation2d gyroAngle = Rotation2d.fromDegrees(90);
        m_poseManager = new PoseManager(
                m_kinematics,
                gyroAngle,
                m_modulePositions,
                new Pose2d(new Translation2d(13, 1), Rotation2d.fromDegrees(90)));
        assertEquals(
                new Pose2d(),
                m_poseManager.updatePose(gyroAngle, m_modulePositions));
    }

    @AfterEach
    void teardown() {
        m_poseManager = null;
    }

}
