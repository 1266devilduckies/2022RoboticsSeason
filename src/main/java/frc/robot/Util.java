package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.interfaces.Gyro;
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
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    motor.setSelectedSensorPosition(0);
    motor.setInverted(false);
    motor.setSensorPhase(false);
  }

  public static void setEncoderDefaultPhoenixSettings(TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 100);
    motor.setSelectedSensorPosition(0);
    motor.setInverted(false);
    motor.setSensorPhase(false);
  }
  public static void setEncoderDefaultPhoenixSettings(VictorSPX motor) {
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.setSensorPhase(false);
  }
  
  public static void rotate(double desiredAngle) {
    /*pseudocode:
    *getting an input of the current rotation the gyro detects
    *do similar function to one in PewPewStart
    *get input, and turn the robot until we are facing the desired angle
    *y is rotation
    */
        
    double deltaAngle = desiredAngle - RobotMap.gyro.getAngle();
        
    if (Math.abs(deltaAngle) > 5) {//if we are outside the acceptable range
      if (deltaAngle >= 0) {//if deltaAngle is positive
        //turn right
        Robot.turnY = 1.0;
      }
      if (deltaAngle < 0) {//if deltaAngle is negative 
          //turn left
          Robot.turnY = -1.0;
      }
    }//if outside range
    //if we are inside the range, no correction
  }//method

  //temporary - move at speed
  public static void move(double desiredRPM){
    /*psuedocode
    *get desired speed from
    * figure out difference between desired speed and Talon Encoders detected speed
    * increase or decrease moveX depending on difference
    */
    
    double deltaSpeed = desiredRPM - (RobotMap.PewPewMotor1.getSelectedSensorVelocity(0)/3);
    
    if (deltaSpeed >= 0) {//if deltaSpeed is positive
      //turn right
      Robot.moveX = 1.0;
    }
    if (deltaSpeed < 0) {//if deltaSpeed is negative 
        //turn left
        Robot.moveX = -1.0;
    }
  }

}