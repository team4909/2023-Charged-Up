package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static final boolean SIM = RobotBase.isSimulation();
    public static final double PERIODIC_LOOP_DURATION = 0.02;

    public static final class VisionConstants {

        public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

        public static final String CAMERA_NAME = "limelight-4909";
        public static final double CAMERA_FOV_DEGREES = Math.hypot(59.6, 49.7);
        public static final double MAX_LED_RANGE_METERS = 9000d; // PhotonVision says set to 9000+ if not using LEDs
        public static final int CAMERA_RESOLUTION_WIDTH = 320;
        public static final int CAMERA_RESOLUTION_HEIGHT = 240;

        public static final double ROBOT_SIDE = Units.inchesToMeters(26);
        public static final double ROBOT_HEIGHT = Units.inchesToMeters(36);

        public static final double CAMERA_PITCH_RADIANS = 0d;
        public static final double CAMERA_HEIGHT_METERS_Z = 1.5;
        public static final double CAMERA_SIDE_Y = 0d;

        /*
         * Bottom back right corner is the origin in OUR coords (x,y,z) -> (Y,Z,X)
         * Forward: +x, Left: +y, Up: +z
         */
        public static final double CAMERA_X = Units.inchesToMeters(24);
        public static final double CAMERA_Y = Units.inchesToMeters(13);
        public static final double CAMERA_Z = Units.inchesToMeters(25);

        public static final Transform3d ROBOT_TO_CAM_DIST = new Transform3d(
                new Translation3d(Units.inchesToMeters(13.0), Units.inchesToMeters(4.75), Units.inchesToMeters(4.0)),
                new Rotation3d(0.0, 0.0, 0.0)

        );

    }
}
