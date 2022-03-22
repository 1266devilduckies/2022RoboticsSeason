package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//RobotMap.java just makes the motors through TalonFX (the actual motors)
//When we start working with a new motor, we basically 'initialize' it here -JM
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotMap {

  public static WPI_TalonFX MainLeftMotorBack;
  public static WPI_TalonFX MainLeftMotorFront;
  public static WPI_TalonFX MainRightMotorBack;
  public static WPI_TalonFX MainRightMotorFront;
  public static VictorSPX IntakeMotor1;
  public static WPI_TalonFX PewPewMotor1;
  public static WPI_TalonFX PewPewMotor2;
  public static WPI_TalonFX FeederMotor;
  public static VictorSPX Climber1;
  public static VictorSPX Climber2;
  public static double avgPositionRaw;
  public static double avgPositionInMeters;
  public static boolean inFiringCoroutine;
  public static long timeSinceStartedBeingReleasedForShooter = -1;
  public static long timeSinceStartedBeingReleasedForSolenoids = -1;
  public static long climberDelay = 1000; //in milliseconds
  public static ADXRS450_Gyro gyro;
  public static Solenoid pneumaticSingleSolenoid;
  public static Compressor pcmCompressor;
  public static boolean isAligningCoroutine = false;
  public static boolean fullShooterPower = true;
  public static boolean inAutonomous = false;
  public static double angle = 0.0;// angle for robot to align to when in aligining command
  public static boolean angleMode = false; // false means to target to the limelight
  // The robot's drive
  public static DifferentialDrive m_drive;
  /*
   * to change the kF go in phoenix tuner > control > percent output
   * fiddle with it until when you click self test snapshot the velocity is close
   * to the thing you want
   * then convert the percent from self test snapshot to decimal and put that as
   * the first multiple in front of 1023.0.
   * set the velocity from self test snapshot into the velocityTarget variable
   */
  public static double velocityTarget = 13250.0;// 14000.0;
  public static double velocityFeeder = 13250.0;
  public static double kF = 0.045;// .0455
  public static double kP = 0.02; // .02
  public static double kFIndex = 0.045;
  public static double kPIndex = 0.02;
  public static double kPAligner = 0.04;
  public static double kDAligner = 0.0;
  public static PIDController alignerController = new PIDController(kPAligner, 0.0, kDAligner);
  public static double kF2 = (0.6343 * 1023.0) / velocityTarget;
  public static double kP2 = 0.0299999714;
  public static double kI2 = 9.98973846E-05;
  public static double kD2 = 0.03999996;
  public static int numOfTogglesOnSolenoids = 0;
  public static boolean reachedGoal = false;
  public static boolean turnedaround = false;
  public static boolean shotFirstShotInAuto = false;
  public static double tankDriveInPlaceError = 0.0;
  public static boolean pilotDisabled = false;
  public static double overrideVelocity;
  public static double limeLightDistance;

  public static final double ksVolts = 0.67766;
  public static final double kvVoltSecondsPerMeter = 2.2804;
  public static final double kaVoltSecondsSquaredPerMeter = 0.6814;

  public static final double kMaxSpeedMetersPerSecond = 1.75;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1.75;
  public static final double kPDriveVel = 3.473;

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kTrackwidthMeters = 0.762;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      kTrackwidthMeters);

  public static final int kEncoderCPR = 2048;
  public static final double kWheelDiameterMeters = 0.1524;
  public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

  public static void init() {

    MainLeftMotorBack = new WPI_TalonFX(0);
    MainLeftMotorFront = new WPI_TalonFX(1);
    MainRightMotorBack = new WPI_TalonFX(3);
    MainRightMotorFront = new WPI_TalonFX(2);
    m_drive = new DifferentialDrive(RobotMap.MainLeftMotorBack,
        RobotMap.MainRightMotorBack);

    inFiringCoroutine = false;
    IntakeMotor1 = new VictorSPX(4);
    FeederMotor = new WPI_TalonFX(9);
    PewPewMotor1 = new WPI_TalonFX(5);
    PewPewMotor2 = new WPI_TalonFX(8);
    Climber1 = new VictorSPX(12);
    Climber2 = new VictorSPX(13);
    Climber2.set(ControlMode.Follower, 12);
    PewPewMotor1.set(ControlMode.Follower, 8);
    MainLeftMotorFront.set(ControlMode.Follower, 0);
    MainRightMotorFront.set(ControlMode.Follower, 3);
    pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    pneumaticSingleSolenoid = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);
    pneumaticSingleSolenoid.set(false);
    gyro = new ADXRS450_Gyro();
    // AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
    /*
     * TalonFXSimCollection fx_simshooter1 = PewPewMotor1.getSimCollection();
     * TalonFXSimCollection fx_simshooter2 = PewPewMotor2.getSimCollection();
     * VictorSPXSimCollection fx_intake1 = IntakeMotor1.getSimCollection();
     * VictorSPXSimCollection fx_feeder1 = FeederMotor.getSimCollection();
     */
  }
}