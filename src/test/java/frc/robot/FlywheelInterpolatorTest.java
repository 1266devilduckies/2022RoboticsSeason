package frc.robot;

import static org.junit.Assert.*;
import org.junit.Test;

import edu.wpi.first.math.util.Units;

public class FlywheelInterpolatorTest {
    @Test
    public void testFindRangeIdx() {
        assertEquals(0, FlywheelInterpolator.findRangeIdx(Units.inchesToMeters(102)));
    }

    @Test
    public void testProcess() {
        double distance = Units.feetToMeters(17.5);
        int idx = FlywheelInterpolator.findRangeIdx(distance);
        double rpm = FlywheelInterpolator.interpolateDataFromIdx(Constants.flywheelRPMData, idx, distance);
        System.out.println(rpm);
    }
}
