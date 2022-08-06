package frc.robot;

public class GearUtil {
  //Gear ratio must be a reduction, CPR means ticks per revolution
  
  public static double encoderTicksToMeters(double ticks, double gearRatio, double CPR, double wheelRadius) {
    double numAxleRotations = ticks/CPR;
    double numRotationsOnDriverGear = numAxleRotations/gearRatio;
    return numRotationsOnDriverGear*2*Math.PI*wheelRadius;
  }
  public static double metersToEncoderTicks(double meters, double gearRatio, double CPR, double wheelRadius) {
   return (meters/(2*Math.PI*wheelRadius))*gearRatio*CPR;
  }
  public static double metersPerSecondToEncoderTicksPer100ms(double metersPerSecond, double gearRatio, double CPR, double wheelRadius) {
    metersPerSecond /= 10.0;
    return metersToEncoderTicks(metersPerSecond, gearRatio, CPR, wheelRadius);
  }
  public static double EncoderTicksPer100msToMetersPerSecond(double ticksPer100ms, double gearRatio, double CPR, double wheelRadius) {
    return encoderTicksToMeters(ticksPer100ms*10.0, gearRatio, CPR, wheelRadius);
  }
  public static double RPMToEncoderTicksPer100ms(double rpm, double gearRatio, double CPR) {
    return ((rpm/60.0) * CPR * gearRatio) / 10.0;
  }

public static double EncoderTicksPer100msToRPM(double velocity, double gearRatio, double CPR) {
    return (((velocity*10.0)/CPR)/gearRatio) * 60.0;
}
}
