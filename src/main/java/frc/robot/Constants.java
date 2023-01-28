package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.swervelib.ModuleConfiguration;
import frc.lib.swervelib.SdsModuleConfigurations;
import frc.lib.swervelib.Mk4iSwerveModuleHelper.GearRatio;

public final class Constants {

    public static final boolean SIM = RobotBase.isSimulation();
    public static final double PERIODIC_LOOP_DURATION = 0.02;

    public static final class DrivetrainConstants {
        // The left-to-right distance between the drivetrain wheels, should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(32);
        // The front-to-back distance between the drivetrain wheels, should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(32);

        public static final String CANBUS = "CANivore1";
        public static final double FALCON_500_FREE_SPEED = 6380d;
        public static final double TICKS_PER_ROTATION = 2048d;
        public static final double MAX_DRIVETRAIN_SPEED = Units.feetToMeters(16.3); //From SDS
        public static final GearRatio GEAR_RATIO = GearRatio.L2;
        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2;
        public static final double PRECISE_SPEED_SCALE = 0.2;
        public static final double DEFAULT_TIMEOUT = 15d; // For auto trajectories

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.99;//Units.rotationsToRadians(-307d / TICKS_PER_ROTATION);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.787;//Units.rotationsToRadians(-242.5d / TICKS_PER_ROTATION);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.81;//Units.rotationsToRadians(-307d / TICKS_PER_ROTATION);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.81;//Units.rotationsToRadians(-306.7d / TICKS_PER_ROTATION);

        //CAN IDS
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
