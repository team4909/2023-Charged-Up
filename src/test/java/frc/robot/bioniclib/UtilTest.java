package frc.robot.bioniclib;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import edu.wpi.first.hal.HAL;
import frc.lib.bioniclib.Util;

@TestInstance(Lifecycle.PER_CLASS)
public class UtilTest {

    @BeforeAll
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @ParameterizedTest
    @CsvSource({ "0, 0, 0", "5, -1, 7", "0, 0, 1.1", "15.4, 9, 15.4", "-0.05, 0.02, 0.05" })
    void inToleranceInBounds(double val, double min, double max) {
        assertTrue(Util.inRange(val, min, max));
    }

    @ParameterizedTest
    @CsvSource({ "5, 1, 4", "4.4, 5, 9", "-9.9, -5.4, -1.1", "-3, -9.85, -5.54" })
    void inToleranceOutOfBounds(double val, double min, double max) {
        assertFalse(Util.inRange(val, min, max));
    }

}
