package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
      public static void setEncodersDefaultPhoenixSettings() {
        RobotMap.MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
        RobotMap.MainLeftMotorFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
        RobotMap.MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
        RobotMap.MainRightMotorFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

        RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0,0,100);
        RobotMap.MainLeftMotorFront.setSelectedSensorPosition(0,0,100);
        RobotMap.MainRightMotorBack.setSelectedSensorPosition(0,0,100);
        RobotMap. MainRightMotorFront.setSelectedSensorPosition(0,0,100);
    
        RobotMap.MainLeftMotorBack.setInverted(false);
        RobotMap.MainLeftMotorFront.setInverted(false);
        RobotMap.MainRightMotorBack.setInverted(false);
        RobotMap.MainRightMotorFront.setInverted(false);

        RobotMap.MainLeftMotorBack.setSensorPhase(false);
        RobotMap.MainLeftMotorFront.setSensorPhase(false);
        RobotMap.MainRightMotorBack.setSensorPhase(false);
        RobotMap.MainRightMotorFront.setSensorPhase(false);
      }
}
