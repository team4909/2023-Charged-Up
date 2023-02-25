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

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 282.7;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 186.2;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 251.2;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 91.2;

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
    
    public static final class ClawConstants {

        public static final double kP = 6;

        public static final double OUTPUT_LIMIT = 0.5;
    }

    public static final class IntakeConstants {
        public static final double kG = 0.66;

        public static final double DEGREE_RANGE = 110d; // from cad
        public static final double TICK_RANGE = 11.428633; // emperically measured
        public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;
    }

    public static final class WristConstants {

        public static final double ZERO_TIME = 3; // seconds

        public static final double kS = 0.01;
        public static final double kP = 0.035;

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A4%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A7.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A105%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A93%2C%22u%22%3A%22deg%22%7D&iterationLimit=20000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A14.2857%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kG = 0.95;
        public static final double kV = 0.28;
        public static final double kA = 0.02;

        public static final double OUTPUT_LIMIT = 0.15;
        public static final double DEGREE_RANGE = 191d; // from cad
        public static final double TICK_RANGE = 7.976211547851562; // emperically measured
        public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;

        public static final class ElevatorConstants {

            public static final int LEFT_MOTOR = 9;
            public static final int RIGHT_MOTOR = 10;

            public static final double ELEVATOR_KP = 0.15;
            public static final double ELEVATOR_KD = 0.1;
            public static final double PEAK_OUTPUT = 0.25;

            public static final double METER_RANGE = Units.inchesToMeters(42); // from cad
            public static final double TICK_RANGE = 28500d; // emperically measured
            public static final double METERS_PER_TICK = METER_RANGE / TICK_RANGE;

            public static final double BOTTOM_SETPOINT = 0d;
            public static final double MID_CONE_SETPOINT = 13_982d * METERS_PER_TICK;
            public static final double MID_CUBE_SETPOINT = 13_908d * METERS_PER_TICK;;
            public static final double TOP_SETPOINT = 28_500d * METERS_PER_TICK;;
        }
    }
}
