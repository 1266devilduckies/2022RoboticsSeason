package frc.robot;
import java.io.IOException;

//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//RobotMap.java just makes the motors through TalonFX (the actual motors)
//When we start working with a new motor, we basically 'initialize' it here -JM

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap{
  
  

  public static TalonFX MainLeftMotorBack;
  public static TalonFX MainLeftMotorFront;
  public static TalonFX MainRightMotorBack;
  public static TalonFX MainRightMotorFront;
  public static TalonFX IntakeMotor1;
  public static TalonFX PewPewMotor1;
  public static TalonFX PewPewMotor2;
<<<<<<< HEAD
  public static double PewPewMotor1RPM;
  public static double PewPewMotor2RPM;
=======
  public static double PewPewMotor1VelocityEstimate = 0.5;
>>>>>>> vision
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
  public static boolean inSubroutine;
  public static PneumaticsControlModule pcm;
  public static PowerDistribution pdp;
  public static ADXRS450_Gyro gyro;
  final public static double velocityTarget = 7250.0;//max velocity is 21750 ticks / 100 ms
  final public static double velocityThreshold = 10.0; 

  public static void init(){
    MainLeftMotorBack = new TalonFX(0);
    MainLeftMotorFront = new TalonFX(1);
    MainRightMotorBack = new TalonFX(2);
    MainRightMotorFront = new TalonFX(3);
    //IntakeMotor1 = new TalonFX(4);
<<<<<<< HEAD
    //PewPewMotor1 = new TalonFX(5);
=======
    PewPewMotor1 = new TalonFX(5);
>>>>>>> vision
    //PewPewMotor2 = new TalonFX(8);
    pcm = new PneumaticsControlModule(6);
    pdp = new PowerDistribution(7, ModuleType.kCTRE);
    gyro = new ADXRS450_Gyro();
  }
}
