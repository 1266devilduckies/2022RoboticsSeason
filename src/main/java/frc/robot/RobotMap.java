package frc.robot;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//RobotMap.java just makes the motors through TalonFX (the actual motors)
//When we start working with a new motor, we basically 'initialize' it here -JM
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
  public static TalonSRX FeederMotor;
  //from tests we found that it is around 30% speed, so the closer we can get before iteration the better
  public static double PewPewMotor1VelocityEstimate = 0.3;
  public static double PewPewMotor2VelocityEstimate = 0.3;
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
  public static int checkerBoardForShooter = 0;
  public static boolean inFiringCoroutine;
  public static long timeSinceStartedBeingReleased = -1;
  public static PneumaticsControlModule pcm;
  public static PowerDistribution pdp;
  public static ADXRS450_Gyro gyro;
  public static double velocityTarget = 14196.0;//max velocity is 21750 ticks / 100 ms, target is 7259 but mathmatically it 7250. from tests we found that 11549 worked
  public static double currentSpeed = 0.0;
  public static double speed = SmartDashboard.getNumber("shooter motor speed", 0.0);
  public static double kF = (0.6343 * 1023.0) / velocityTarget;

  public static void init(){
    MainLeftMotorBack = new TalonFX(0);
    MainLeftMotorFront = new TalonFX(1);
    MainRightMotorBack = new TalonFX(2);
    MainRightMotorFront = new TalonFX(3);
    FeederMotor = new TalonSRX(9);
    inFiringCoroutine = false;
    IntakeMotor1 = new TalonFX(4);
    PewPewMotor1 = new TalonFX(5);
    PewPewMotor2 = new TalonFX(8);
    PewPewMotor1.set(ControlMode.Follower, 8);
    MainLeftMotorFront.set(ControlMode.Follower, 0);
    MainRightMotorFront.set(ControlMode.Follower, 2);
    pcm = new PneumaticsControlModule(6);
    pdp = new PowerDistribution(7, ModuleType.kCTRE);
    gyro = new ADXRS450_Gyro();
  }
}
