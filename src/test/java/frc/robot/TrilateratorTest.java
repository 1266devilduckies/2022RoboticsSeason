package frc.robot;

import org.junit.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class TrilateratorTest {
    @Test
    public void testInput() {
        Translation2d result = Trilaterator.findOrigin(
            new Translation3d(8.8,9.36,0.),
            new Translation3d(3.,6.3,0.),
            69420.,
            true,
            5.7,
            6.77);
        System.out.println(result.toString());
    }
}
