package frc.robot;

import static edu.wpi.first.wpilibj.CounterBase.EncodingType.k1X;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.commands.PewPewStart;
import frc.robot.EncoderSetter;
import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.hal.simulation.EncoderDataJNI;
//import edu.wpi.first.hal.EncoderJNI;
//import edu.wpi.first.wpilibj.simulation.EncoderSim;
//import edu.wpi.first.wpilibj.drive.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.wpilibj.drive.*;
//import frc.robot.subsystems.*;
//import edu.wpi.first.wpilibj.*;

//import java.io.IOException;
//import java.nio.file.Path;

//This is basically our main class, we just don't use Main.java for clarity (i guess) -JM

public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static DriveSubsystem m_robotDrive;
  public static Intake intake;
  public static Shooter shooter;
  public static double turnY;
  public static double moveX;
  // in milliseconds
  public long startAutoTime;
  public long currentAutoTime = 0;

  // test trajectory
  Trajectory trajectory = new Trajectory();

  /*
   * //CALIBRATE VALUE TO OUR ROBOT LATER
   * public static final double ksVolts = 0.22;
   * public static final double kvVoltSecondsPerMeter = 1.98;
   * public static final double kaVoltSecondsSquaredPerMeter = 0.2;
   * public static final double kPDriveVel = 8.5;
   * public static final double kTrackwidthMeters = 0.71;
   * public static final double kMaxSpeedMetersPerSecond = 3;
   * public static final double kMaxAccelerationMetersPerSecondSquared = 3;
   * public static final DifferentialDriveKinematics kDriveKinematics = new
   * DifferentialDriveKinematics(kTrackwidthMeters);
   * 
   * public static final double kRamseteB = 2;
   * public static final double kRamseteZeta = 0.7;
   */

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    RobotMap.init();
    m_robotDrive = new DriveSubsystem();
    drivetrain = new Drivetrain();
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorBack);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorFront);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorBack);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorFront);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor1);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor2);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.FeederMotor);
    RobotMap.PewPewMotor2.setInverted(true);
    RobotMap.PewPewMotor1.setInverted(false);
    RobotMap.IntakeMotor1.setInverted(false);

    // configure the PID

    RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor1.config_kI(0, RobotMap.kI);
    RobotMap.PewPewMotor1.config_kD(0, RobotMap.kD);
    RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor2.config_kI(0, RobotMap.kI);
    RobotMap.PewPewMotor2.config_kD(0, RobotMap.kD);

    RobotMap.gyro.calibrate();
    intake = new Intake();
    shooter = new Shooter();
    JoystickController.Init();
    // get auto path json
    SmartDashboard.putData("Auto mode", m_chooser);
    String trajectoryJSON = "paths/Auto1.wpilib.json";

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    /*
     * if (!Preferences.containsKey("kP Aligner PID")) {
     * Preferences.setDouble("kP Aligner PID", RobotMap.kPAligner);
     * }
     * if (!Preferences.containsKey("kI Aligner PID")) {
     * Preferences.setDouble("kI Aligner PID", RobotMap.kIAligner);
     * }
     * if (!Preferences.containsKey("kD Aligner PID")) {
     * Preferences.setDouble("kD Aligner PID", RobotMap.kDAligner);
     * }
     */
  }

  @Override
  public void robotPeriodic() {
    /*
     * double sdkP = Preferences.getDouble("kP Aligner PID", RobotMap.kPAligner);
     * double sdkI = Preferences.getDouble("kI Aligner PID", RobotMap.kIAligner);
     * double sdKd = Preferences.getDouble("kD Aligner PID", RobotMap.kDAligner);
     * if (RobotMap.kPAligner != sdkP |
     * RobotMap.kIAligner != sdkI |
     * RobotMap.kDAligner != sdKd) {
     * RobotMap.kPAligner = sdkP;
     * RobotMap.kIAligner = sdkI;
     * RobotMap.kDAligner = sdKd;
     * RobotMap.alignerPIDController = new PIDController(sdkP, sdkI, sdKd);
     * }
     */
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    startAutoTime = System.currentTimeMillis();
    SequentialCommandGroup m_autonomousCommand = getAutonoumous(1);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainLeftMotorFront.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorFront.setSelectedSensorPosition(0);
    RobotMap.FeederMotor.setSelectedSensorPosition(0);

    drivetrain.arcadeDriveVoltage(0., 0., 0.75, -0.75);
    Scheduler.getInstance().add(new BetterKearnyDriving());

  }

  @Override
  public void teleopPeriodic() {
    // periodic events
    limeLightDataFetcher.fetchData();
    if (RobotMap.timeSinceStartedBeingReleasedForSolenoids != -1) {
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForSolenoids) >= 1000) {
        RobotMap.IntakeMotor1.set(ControlMode.PercentOutput, 1.0);
      }
    }
    // the overall interval for this should be adjusted depending on how good the
    // PIDF can recover
    if (RobotMap.inFiringCoroutine) {
      long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
      long interval = 1000;
      if (dt >= interval * 5.5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0.0);
        RobotMap.inFiringCoroutine = false;
      } else if (dt >= interval * 5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
      } else if (dt >= interval * 3) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
      } else if (dt >= interval * 2.5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
      } else {
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, RobotMap.velocityTarget);
      }
    }

    if (RobotMap.pcmCompressor.getCurrent() > 130.0) {
      RobotMap.pcmCompressor.disable();

    } else if (RobotMap.pcmCompressor.getCurrent() < 90.0) {
      RobotMap.pcmCompressor.enableDigital();
    }
    if (RobotMap.isAligningCoroutine) {
      if (limeLightDataFetcher.seeIfTargetsExist() == 1.0) {
        double pidOutput = RobotMap.alignerPIDController.calculate(limeLightDataFetcher.getdegRotationToTarget(), 0.0);
        drivetrain.arcadeDriveVoltage(0.0, Math.max(-1.0, Math.min(1.0, pidOutput)), 0.0, 1.0);
      } else {
        drivetrain.arcadeDriveVoltage(0.0, 0.0, 0.0, 1.0);
      }
    }
    // logging data
    SmartDashboard.putBoolean("in coroutine", RobotMap.inFiringCoroutine);
    SmartDashboard.putNumber("gyro rotation", RobotMap.gyro.getAngle());
    SmartDashboard.putNumber("diffrence x", limeLightDataFetcher.getdegRotationToTarget());
    SmartDashboard.putNumber("difference y", limeLightDataFetcher.getdegVerticalToTarget());

    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousPeriodic() {
    double error = -RobotMap.gyro.getRate();

    /*
     * //https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/
     * encoders-software.html
     * //other side is flipped internally
     * if (RobotMap.avgPositionInMeters < 2.3) {
     * //face of intake direction is negative
     * drivetrain.arcadeDriveVoltage(-0.2,.5 - 1 * error, 0.75, -0.75);
     * } else {
     * drivetrain.arcadeDriveVoltage(0, .5 - 1 * error, 0.75, -0.75);
     * }
     */

    currentAutoTime = startAutoTime - System.currentTimeMillis();

    /*
     * if(currentAutoTime >= milliseconds){
     * //intake
     * }
     * else if(currentAutotime >= milliseconds){
     * //shoot
     * }
     */

  }

  public SequentialCommandGroup getAutonoumous(int num) {
    // ITZ PATHWEAVER LAND from here on out
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveSubsystem.DriveConstants.ksVolts,
            DriveSubsystem.DriveConstants.kvVoltSecondsPerMeter,
            DriveSubsystem.DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveSubsystem.DriveConstants.kDriveKinematics,
        10);

    TrajectoryConfig config = new TrajectoryConfig(
        DriveSubsystem.DriveConstants.kMaxSpeedMetersPerSecond,
        DriveSubsystem.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveSubsystem.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(DriveSubsystem.DriveConstants.kRamseteB, DriveSubsystem.DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveSubsystem.DriveConstants.ksVolts,
            DriveSubsystem.DriveConstants.kvVoltSecondsPerMeter,
            DriveSubsystem.DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveSubsystem.DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveSubsystem.DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveSubsystem.DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

  }

  @Override
  public void testPeriodic() {
  }

}
