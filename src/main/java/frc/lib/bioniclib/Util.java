package frc.lib.bioniclib;

/**
 * Assortment of utility functions
 * 
 * @author Mihir Patankar
 * @author Billerica Bionics FRC 4909
 */
public final class Util {

    /**
     * @param value
     * @param minValue Lower bound for the value
     * @param maxValue Upper bound for the value
     * @return True if the value is between the lower and upper bounds
     */
    public static boolean inRange(double value, double minValue, double maxValue) {
        if (value >= 0)
            return value <= maxValue && value >= minValue;
        return value <= minValue && value >= maxValue;
    }
}
