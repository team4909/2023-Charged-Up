package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static final boolean SIM = RobotBase.isSimulation();
  public static final double PERIODIC_LOOP_DURATION = 0.02;
  public static final double NOMINAL_VOLTAGE = 12d;
  public static final String CANFD_BUS = "CANivore1";

  public static final class VisionConstants {

    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    public static final double MAX_X_DEVIATION = 1.0;
    public static final double MAX_Y_DEVIATION = 1.0;
    public static final double MIN_AREA_ONE_TARGET = 0.5;
    public static final double MIN_AREA_MULTIPLE_TARGETS = 0.4;

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.32385, -0.19685, 0.71755),
        new Rotation3d(Math.toRadians(0.0), Math.toRadians(-22.0), Math.toRadians(0.0)));
  }

  public static final class DrivetrainConstants {
    // The left-to-right distance between the drivetrain wheels, should be measured
    // from center to center.
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.5);
    // The front-to-back distance between the drivetrain wheels, should be measured
    // from center to center.
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.5);

    public static final double FALCON_500_FREE_SPEED = 6380d;
    public static final double TICKS_PER_ROTATION = 2048d;
    public static final double WHEEL_DIAMETER = 0.10033;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double MAX_DRIVETRAIN_SPEED = 4.2;
    public static final double PRECISE_SPEED_SCALE = 0.2;
    public static final double DEFAULT_TIMEOUT = 15d; // For auto trajectories
    public static final double DEADBAND = 0.05;

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 282.7;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 185.2;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 251.2;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 91.8;

    public static final double DRIVE_kP = 0.07;
    public static final double DRIVE_kS = 0.26015 / 12d;
    public static final double DRIVE_kV = 2.5039 / 12d;
    public static final double DRIVE_kA = 0.99695 / 12d;

    public static final double TURN_kP = 0.3;

    public static final double X_FOLLOWING_kP = 1.8;
    public static final double Y_FOLLOWING_kP = 1.8;
    public static final double THETA_FOLLOWING_kP = 3;

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

    public static final double kP = 7.5;

    public static final double OUTPUT_LIMIT = 0.5;
  }

  public static final class IntakeConstants {

    public static final int LEFT_PIVOT_MOTOR = 3;
    public static final int FRONT_ROLLER_MOTOR = 1;
    public static final int BACK_ROLLER_MOTOR = 4;

    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A4%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A7.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A105%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A93%2C%22u%22%3A%22deg%22%7D&iterationLimit=20000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A14.2857%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D

    public static final double kP = 0.01;
    public static final double kG = 0.7;

    public static final double OUTPUT_LIMIT = 0.5;
    public static final double DEGREE_RANGE = 110d; // from cad
    public static final double TICK_RANGE = 11.428633; // emperically measured
    public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;

    public static final double RETRACTED_SETPOINT = 110d;
    public static final double CUBE_SETPOINT = 16d;
    public static final double CONE_SETPOINT = 11d;
    public static final double HANDOFF_SETPOINT = 73d;
    public static final double SPIT_CONE_SETPOINT = 60d;

    public static final class SIM {
      public static final double GEARING = 7d;
      public static final double ARM_LENGTH = Units.inchesToMeters(12);
      private static final double ARM_WEIGHT = Units.lbsToKilograms(7.5);
      // Approx, using 1/3 * mr^2 (moi for a rod about end)
      public static final double MOI = (1d / 3d) * ARM_WEIGHT * Math.pow(ARM_LENGTH, 2);
    }
  }

  public static final class WristConstants {

    public static final double kP = 0.035;
    public static final double kG = 0.85;

    public static final double OUTPUT_LIMIT = 0.25;
    public static final double DEGREE_RANGE = 191d; // from cad
    public static final double TICK_RANGE = 7.976211547851562; // emperically measured
    public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;

    public static final class SIM {
      public static final double GEARING = 14.2857;
      public static final double ARM_LENGTH = Units.inchesToMeters(15);
      private static final double ARM_WEIGHT = Units.lbsToKilograms(4);
      // Approx, using 1/3 * mr^2 (moi for a rod about end)
      public static final double MOI = (1d / 3d) * ARM_WEIGHT * Math.pow(ARM_LENGTH, 2);

    }
  }

  public static final class CubeShooterConstants {
    public static final int PIVOT_MOTOR = 17;
    public static final int TOP_ROLLER_MOTOR = 15;
    public static final int BOTTOM_ROLLER_MOTOR = 16;

    public static final double RETRACTED_SETPOINT = 104;
    public static final double DOWN_SETPOINT = 5.0;
    public static final double CUBE_MID = 75;

    public static final double OUTPUT_LIMIT = 1.0;
    public static final double DEGREE_RANGE = 111.0; // from cad
    public static final double TICK_RANGE = 16.545471; // emperically measured
    public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;

    // https://www.reca.lc/arm?armMass=%7B%22s%22%3A11.3%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A11%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=65&endAngle=%7B%22s%22%3A100%2C%22u%22%3A%22deg%22%7D&iterationLimit=20000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A30%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
    public static final double kP = 0.011; // 0.04 / 4;
    public static final double kD = 0.9;
    public static final double kG = 0.25;
  }

  public static final class ElevatorConstants {

    public static final int LEFT_MOTOR = 9;
    public static final int RIGHT_MOTOR = 10;

    public static final double kP = 0.022;
    // Gains were recalced
    public static final double kS = 0.40921;
    public static final double kG = 0.12654;
    public static final double kV = 1.3374;
    public static final double kA = 0.031771;
    public static final double OUTPUT_LIMIT = 0.25;
    public static final double MOTION_CRUISE_VELOCITY = 4.7; // m/s
    public static final double MOTION_ACCELERATION = 10.1; // m/s^2

    public static final double METER_RANGE = Units.inchesToMeters(42); // from cad
    public static final double TICK_RANGE = 28500d; // emperically measured
    public static final double METERS_PER_TICK = METER_RANGE / TICK_RANGE;

    public static final double BOTTOM_SETPOINT = 0d;
    public static final double MID_CONE_SETPOINT = 13_982d * METERS_PER_TICK;
    public static final double MID_CUBE_SETPOINT = 13_908d * METERS_PER_TICK;
    public static final double TOP_SETPOINT = 1.03;
    public static final double DOUBLE_SUBSTATION_SETPOINT = 28_500d * METERS_PER_TICK;
    public static final double SUBSTATION_SETPOINT = 1234 * METERS_PER_TICK;

    public static final class SIM {
      public static final double GEARING = 4d; // Wild Guess
      public static final double DRUM_RADIUS = Units.inchesToMeters(1d); // Wild Guess
      public static final double CARRIAGE_MASS = Units.lbsToKilograms(12.5); // Estimate
      public static final double RETRACTED_LENGTH = Units.inchesToMeters(30d);
    }
  }
}
