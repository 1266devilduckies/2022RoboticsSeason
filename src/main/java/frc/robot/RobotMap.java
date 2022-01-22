package frc.robot;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//RobotMap.java just makes the motors through TalonFX (the actual motors)
//When we start working with a new motor, we basically 'initialize' it here -JM

public class RobotMap{
  
  

  public static TalonFX MainLeftMotorBack;
  public static TalonFX MainLeftMotorFront;
  public static TalonFX MainRightMotorBack;
  public static TalonFX MainRightMotorFront;
  public static TalonFX IntakeMotor1;
  public static TalonFX PewPewMotor1;
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
 

  public static void init(){
    MainLeftMotorBack = new TalonFX(0);
    MainLeftMotorFront = new TalonFX(1);
    MainRightMotorBack = new TalonFX(2);
    MainRightMotorFront = new TalonFX(3);
    MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainLeftMotorFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    MainLeftMotorBack.setSelectedSensorPosition(0,0,100);
    MainLeftMotorFront.setSelectedSensorPosition(0,0,100);
    MainRightMotorBack.setSelectedSensorPosition(0,0,100);
    MainRightMotorFront.setSelectedSensorPosition(0,0,100);
    
    //MainLeftMotorBack.setInverted(true);
    //MainLeftMotorFront.setInverted(true);

    MainLeftMotorBack.setSensorPhase(false);
    MainLeftMotorFront.setSensorPhase(false);
    MainRightMotorBack.setSensorPhase(false);
    MainRightMotorFront.setSensorPhase(false);

    IntakeMotor1 = new TalonFX(4);
    PewPewMotor1 = new TalonFX(5);

  }
}
