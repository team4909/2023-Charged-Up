package frc.robot;

public final class Constants {

    public static final class ClawConstants {

        public static final double kP = 5;

        public static final double OUTPUT_LIMIT = 0.09;
    }

    public static final class WristConstants {

        public static final double ZERO_TIME = 3; // seconds

        public static final double kS = 0.01;
        public static final double kP = 0.035;

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A4%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A7.5%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A105%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A93%2C%22u%22%3A%22deg%22%7D&iterationLimit=20000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A14.2857%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D
        public static final double kG = 0.95;
        public static final double kV = 0.28;
        public static final double kA = 0.02;

        public static final double OUTPUT_LIMIT = 0.2;
        public static final double DEGREE_RANGE = 191d; // from cad
        public static final double TICK_RANGE = 7.976211547851562; // emperically measured
        public static final double DEGREES_PER_TICK = DEGREE_RANGE / TICK_RANGE;
    }
}
