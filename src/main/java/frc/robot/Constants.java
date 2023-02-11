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

    public static final class DrivetrainConstants {
        // The left-to-right distance between the drivetrain wheels, should be measured
        // from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(32);
        // The front-to-back distance between the drivetrain wheels, should be measured
        // from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(32);

        public static final String CANBUS = "CANivore1";
        public static final double FALCON_500_FREE_SPEED = 6380d;
        public static final double TICKS_PER_ROTATION = 2048d;
        public static final double WHEEL_DIAMETER = 0.10033;
        public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double MAX_DRIVETRAIN_SPEED = Units.feetToMeters(16.3); // From SDS
        public static final double PRECISE_SPEED_SCALE = 0.2;
        public static final double DEFAULT_TIMEOUT = 15d; // For auto trajectories
        public static final double DEADBAND = 0.05;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 426.9;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 78.4;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 250.0;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 317.7;

        // CAN IDS
        public static final int PIGEON_ID = 20;
        public static final int FRONT_LEFT_DRIVE_MOTOR = 7;
        public static final int FRONT_LEFT_TURN_MOTOR = 8;
        public static final int FRONT_LEFT_STEER_ENCODER = 14;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
        public static final int FRONT_RIGHT_TURN_MOTOR = 1;
        public static final int FRONT_RIGHT_STEER_ENCODER = 11;
        public static final int BACK_LEFT_DRIVE_MOTOR = 6;
        public static final int BACK_LEFT_TURN_MOTOR = 5;
        public static final int BACK_LEFT_STEER_ENCODER = 13;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 4;
        public static final int BACK_RIGHT_TURN_MOTOR = 3;
        public static final int BACK_RIGHT_STEER_ENCODER = 12;

    }
}
