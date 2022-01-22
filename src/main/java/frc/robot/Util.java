package frc.robot;
public class Util {
    public static double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / 2048; //is units per rotation for the falcons
        double wheelRotations = motorRotations / 10.0; //10:1 is gear ratio
        double positionMeters = wheelRotations * (2 * Math.PI * .0762); //3 inches in meters is .0762 meters. the wheels radius is 3 inches
        return positionMeters;
      }
}
