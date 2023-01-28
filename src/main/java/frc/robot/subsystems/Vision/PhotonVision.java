package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VisionConstants.*;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase{

    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimation;
    public AprilTagFieldLayout atfl;

    public PhotonVision() {

        camera = new PhotonCamera(CAMERA_NAME);

        try {
            atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        poseEstimation = new PhotonPoseEstimator(
        atfl, // Field layout
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        camera,
        ROBOT_TO_CAM_DIST
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        poseEstimation.setReferencePose(new Pose3d(5, 5, 0, new Rotation3d(0, 0, 0)));
        return poseEstimation.update();
    }
}
