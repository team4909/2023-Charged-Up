package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public final class Constants {

    public static class DrivetrainConstants {
        
    }

    public static class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);
    }
    
    public static class VisionConstants {
        public static final String CAMERA_NAME = "Limelight";

        public static final Transform3d ROBOT_TO_CAM_DIST = new Transform3d(
            new Translation3d(Units.inchesToMeters(13.0), Units.inchesToMeters(4.75), Units.inchesToMeters(4.0)), //distance in meters from the robot to the camera
            new Rotation3d(0.0, 0.0, 0.0) // rotation of camera in relation to the robot 
            //Ideally only pitch is a non-zero value

        );
    }
}
