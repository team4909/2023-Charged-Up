package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.lib.swervelib.ModuleConfiguration;
import frc.lib.swervelib.SdsModuleConfigurations;
import frc.lib.swervelib.Mk4iSwerveModuleHelper.GearRatio;

public final class Constants {

    public static final class ElevatorConstants {

        public static final int LEFT_MOTOR = 9;
        public static final int RIGHT_MOTOR = 10;

        public static final double ELEVATOR_KP = 0.2;
        public static final double ELEVATOR_KD = 0.1;
        public static final double PEAK_OUTPUT = 0.5;

        public static final double BOTTOM_SETPOINT = 0d;
        public static final double MID_CONE_SETPOINT = 17_488d;
        public static final double MID_CUBE_SETPOINT = 17_488d;
        public static final double TOP_SETPOINT = 28_581d;

    }
}
