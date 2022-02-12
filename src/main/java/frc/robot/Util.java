package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Util {
      public static double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double)sensorCounts / 2048; //is units per rotation for the falcons
        double wheelRotations = motorRotations / 10.0; //10:1 is gear ratio
        double positionMeters = wheelRotations * (2 * Math.PI * .0762); //3 inches in meters is .0762 meters. the wheels radius is 3 inches
        return positionMeters;
      }
      public static void updateEncoders() {
        RobotMap.avgPositionRaw = (RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0) + 
        RobotMap.MainLeftMotorFront.getSelectedSensorPosition(0))/2.0;
        RobotMap.avgPositionInMeters = nativeUnitsToDistanceMeters(RobotMap.avgPositionRaw);
      }
      public static void setEncoderDefaultPhoenixSettings(TalonFX motor) {
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
        motor.setSelectedSensorPosition(0);
        motor.setInverted(false);
        motor.setSensorPhase(false);
      }
}
