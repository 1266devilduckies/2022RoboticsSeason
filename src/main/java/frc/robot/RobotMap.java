package frc.robot;
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

public class RobotMap{
  
  

  public static TalonFX MainLeftMotorBack;
  public static TalonFX MainLeftMotorFront;
  public static TalonFX MainRightMotorBack;
  public static TalonFX MainRightMotorFront;
  public static TalonFX IntakeMotor1;
  public static TalonFX PewPewMotor1;
  public static TalonFX PewPewMotor2;
  public static double PewPewMotor1RPM;
  public static double PewPewMotor2RPM;
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
  public static boolean inSubroutine;
  public static PneumaticsControlModule pcm;
  public static PowerDistribution pdp;
 

  public static void init(){
    MainLeftMotorBack = new TalonFX(0);
    MainLeftMotorFront = new TalonFX(1);
    MainRightMotorBack = new TalonFX(2);
    MainRightMotorFront = new TalonFX(3);
    //IntakeMotor1 = new TalonFX(4);
    //PewPewMotor1 = new TalonFX(5);
    //PewPewMotor2 = new TalonFX(8);
    pcm = new PneumaticsControlModule(6);
    pdp = new PowerDistribution(7, ModuleType.kCTRE);
  }
}
