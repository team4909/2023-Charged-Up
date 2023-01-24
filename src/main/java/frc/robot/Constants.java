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

        public static final double FALCON_500_FREE_SPEED = 6380d;
        public static final GearRatio GEAR_RATIO = GearRatio.L2;
        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2;
        public static final double PRECISE_SPEED_SCALE = 0.2;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(307d);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(242.5d);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(307d);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(306.7d);

        //CAN IDS
        public static final int PIGEON_ID = 20;
        public static final int FRONT_LEFT_DRIVE_MOTOR = 7;
        public static final int FRONT_LEFT_STEER_MOTOR = 8;
        public static final int FRONT_LEFT_STEER_ENCODER = 14;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR = 1;
        public static final int FRONT_RIGHT_STEER_ENCODER = 11;
        public static final int BACK_LEFT_DRIVE_MOTOR = 6;
        public static final int BACK_LEFT_STEER_MOTOR = 5;
        public static final int BACK_LEFT_STEER_ENCODER = 13;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 4;
        public static final int BACK_RIGHT_STEER_MOTOR = 3;
        public static final int BACK_RIGHT_STEER_ENCODER = 12;
        
    }
}
