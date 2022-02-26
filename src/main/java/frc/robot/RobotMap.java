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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap {

  public static WPI_TalonFX MainLeftMotorBack;
  public static WPI_TalonFX MainLeftMotorFront;
  public static WPI_TalonFX MainRightMotorBack;
  public static WPI_TalonFX MainRightMotorFront;
  public static VictorSPX IntakeMotor1;
  public static TalonFX PewPewMotor1;
  public static TalonFX PewPewMotor2;
  public static TalonSRX FeederMotor;
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
  public static boolean inFiringCoroutine;
  public static long timeSinceStartedBeingReleasedForShooter = -1;
  public static long timeSinceStartedBeingReleasedForSolenoids = -1;
  public static ADXRS450_Gyro gyro;
  public static DoubleSolenoid pneumaticDoubleSolenoid;
  public static Compressor pcmCompressor;
  public static boolean isAligningCoroutine = false;
  public static PIDController alignerPIDController;
  /*
   * to change the kF go in phoenix tuner > control > percent output
   * fiddle with it until when you click self test snapshot the velocity is close
   * to the thing you want
   * then convert the percent from self test snapshot to decimal and put that as
   * the first multiple in front of 1023.0.
   * set the velocity from self test snapshot into the velocityTarget variable
   */
  public static double velocityTarget = 14300.0;
  final public static double kF = (0.6343 * 1023.0) / velocityTarget;
  final public static double kP = 0.0299999714;
  final public static double kI = 9.98973846E-05;
  final public static double kD = 0.03999996;
  public static double kPAligner = 0.02;
  public static double kIAligner = 0.0;
  public static double kDAligner = 0.0;
  public static int numOfTogglesOnSolenoids = 0;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public static void init() {

    MainLeftMotorBack = new WPI_TalonFX(0);
    MainLeftMotorFront = new WPI_TalonFX(1);
    MainRightMotorBack = new WPI_TalonFX(2);
    MainRightMotorFront = new WPI_TalonFX(3);

    inFiringCoroutine = false;
    IntakeMotor1 = new VictorSPX(4);
    FeederMotor = new TalonSRX(9);
    PewPewMotor1 = new TalonFX(5);
    PewPewMotor2 = new TalonFX(8);
    PewPewMotor1.set(ControlMode.Follower, 8);
    MainLeftMotorFront.set(ControlMode.Follower, 0);
    MainRightMotorFront.set(ControlMode.Follower, 2);
    pcmCompressor = new Compressor(6, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    pneumaticDoubleSolenoid = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 6, 7);
    pneumaticDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    gyro = new ADXRS450_Gyro();
    alignerPIDController = new PIDController(kPAligner, kIAligner, kDAligner);
  }
}
