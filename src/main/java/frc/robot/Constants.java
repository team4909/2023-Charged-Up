package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.lib.swervedrivespecialties.swervelib.ModuleConfiguration;
import frc.lib.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import frc.lib.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

public final class Constants {
    public static final class DrivetrainConstants {
        // The left-to-right distance between the drivetrain wheels, should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(20.768);
        // The front-to-back distance between the drivetrain wheels, should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(20.768);

        public static final double FALCON_500_FREE_SPEED = 6380d;
        public static final GearRatio GEAR_RATIO = GearRatio.L2;
        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L2;
        public static final double PRECISE_SPEED_SCALE = 0.25;
        public static final double AUTO_SPEED_SCALE = 0.1;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0d);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0d);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0d);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0d);

        //CAN IDS
        public static final int PIGEON_ID = 0;
        public static final int FRONT_LEFT_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_STEER_MOTOR = 0;
        public static final int FRONT_LEFT_STEER_ENCODER = 0;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 0;
        public static final int FRONT_RIGHT_STEER_MOTOR = 0;
        public static final int FRONT_RIGHT_STEER_ENCODER = 0;
        public static final int BACK_LEFT_DRIVE_MOTOR = 0;
        public static final int BACK_LEFT_STEER_MOTOR = 0;
        public static final int BACK_LEFT_STEER_ENCODER = 0;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 0;
        public static final int BACK_RIGHT_STEER_MOTOR = 0;
        public static final int BACK_RIGHT_STEER_ENCODER = 0;
        
    }
}
