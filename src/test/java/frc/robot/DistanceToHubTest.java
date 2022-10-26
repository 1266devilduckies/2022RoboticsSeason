package frc.robot;

import org.junit.Test;

import edu.wpi.first.math.util.Units;

public class DistanceToHubTest {
    @Test
    public void seeDistanceFromShot() {
        double ty = -10.08;
        double tx = -1.11;
        double input = ComputerVisionUtil.calculateDistanceToTarget(Constants.limelightHeight, 
        Constants.hubHeight, Units.degreesToRadians(Constants.limelightMountAngle), 
        Units.degreesToRadians(ty), Units.degreesToRadians(-tx));
        System.out.println(input);
    }
}
