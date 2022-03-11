package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.hal.simulation.EncoderDataJNI;
//import edu.wpi.first.hal.EncoderJNI;
//import edu.wpi.first.wpilibj.simulation.EncoderSim;
//import edu.wpi.first.wpilibj.drive.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.util.Units;
import frc.robot.limeLightDataFetcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

//This is basically our main class, we just don't use Main.java for clarity (i guess) -JM

public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static DriveSubsystem m_robotDrive;
  public static Intake intake;
  public static Shooter shooter;
  public static Climber climber;
  public static double turnY;
  public static double moveX;
  // in milliseconds
  public long startAutoTime;
  public long currentAutoTime = 0;
  Field2d m_field = new Field2d();

  // test trajectory
  Trajectory trajectory = new Trajectory();
  public static DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2), // 2 Falcon 500s on each side of the drivetrain.
      10, // Standard AndyMark Gearing reduction.
      2.1, // MOI of 2.1 kg m^2 (from CAD model).
      43.01, // Mass of the robot is 43.01 kg.
      Units.inchesToMeters(3), // Robot uses 3" radius (6" diameter) wheels.
      0.546,
      null);
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
    // EncoderSim simEncoder = new EncoderSim(RobotMap.MainLeftMotorBack);
    RobotMap.PewPewMotor2.setInverted(true);
    RobotMap.PewPewMotor1.setInverted(false);
    RobotMap.IntakeMotor1.setInverted(false);
    RobotMap.MainLeftMotorFront.setInverted(true);
    RobotMap.MainLeftMotorBack.setInverted(true);
    RobotMap.MainRightMotorFront.setInverted(false);
    RobotMap.MainRightMotorBack.setInverted(false);
    RobotMap.FeederMotor.setNeutralMode(NeutralMode.Brake);
    RobotMap.MainLeftMotorFront.configOpenloopRamp(0.5);
    RobotMap.MainLeftMotorBack.configOpenloopRamp(0.5);
    RobotMap.MainRightMotorFront.configOpenloopRamp(0.5);
    RobotMap.MainRightMotorBack.configOpenloopRamp(0.5);

    SmartDashboard.putData("Field", m_field);
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
    climber = new Climber();
    JoystickController.Init();
    // get auto path json

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
    if (RobotMap.inFiringCoroutine) {

      double velocity = 0.0;
      if (RobotMap.fullShooterPower) {
        velocity = RobotMap.velocityTarget;
      } else {
        velocity = RobotMap.velocityTarget / 2; // replace with exact value later
      }

      long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
      long interval = 1000;
      if (dt >= interval * 5.5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
        RobotMap.PewPewMotor2.set(ControlMode.PercentOutput, 0.0);
        RobotMap.inFiringCoroutine = false;
        RobotMap.reachedGoal = false; // for autonomus
        RobotMap.fullShooterPower = true;
      } else if (dt >= interval * 5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
      } else if (dt >= interval * 3) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
      } else if (dt >= interval * 2.5) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
      } else {
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, velocity);
      }
    }
    DriveSubsystem.m_odometry.update(RobotMap.gyro.getRotation2d(),
        EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainLeftMotorBack.getSelectedSensorPosition()),
        EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainRightMotorBack.getSelectedSensorPosition()));
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
    /*
     * startAutoTime = System.currentTimeMillis();
     * SequentialCommandGroup m_autonomousCommand = getAutonoumous(1);
     * 
     * // schedule the autonomous command (example)
     * if (m_autonomousCommand != null) {
     * m_autonomousCommand.schedule();
     * }
     */
    RobotMap.gyro.calibrate();
  }

  @Override
  public void teleopInit() {
    RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainLeftMotorFront.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorFront.setSelectedSensorPosition(0);
    RobotMap.FeederMotor.setSelectedSensorPosition(0);

    RobotMap.m_drive.arcadeDrive(0.0, 0.0);
    Scheduler.getInstance().add(new BetterKearnyDriving());

  }

  @Override
  public void teleopPeriodic() {
    // periodic events

    limeLightDataFetcher.fetchData();
    if (RobotMap.timeSinceStartedBeingReleasedForSolenoids != -1) {
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForSolenoids) >= 0) {
        RobotMap.IntakeMotor1.set(ControlMode.PercentOutput, 1.0);
      }
    }

    if (RobotMap.pcmCompressor.getCurrent() > 130.0) {
      RobotMap.pcmCompressor.disable();

    } else if (RobotMap.pcmCompressor.getCurrent() < 90.0) {
      RobotMap.pcmCompressor.enableDigital();
    }
    // logging data
    SmartDashboard.putBoolean("in coroutine", RobotMap.inFiringCoroutine);
    SmartDashboard.putNumber("gyro rotation", RobotMap.gyro.getAngle());
    SmartDashboard.putNumber("diffrence x", limeLightDataFetcher.getdegRotationToTarget());
    SmartDashboard.putNumber("difference y", limeLightDataFetcher.getdegVerticalToTarget());

    Scheduler.getInstance().run();

  }

  @Override
  public void simulationPeriodic() {
    /* Pass the robot battery voltage to the simulated Talon FXs */
    Scheduler.getInstance().add(new BetterKearnyDriving());
    // cod
    /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     *
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     *
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    m_driveSim.setInputs(RobotMap.MainRightMotorBack.getMotorOutputPercent(),
        -RobotMap.MainLeftMotorBack.getMotorOutputPercent());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.update(0.001);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    RobotMap.MainRightMotorBack.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainLeftMotorFront.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainRightMotorFront.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainLeftMotorBack.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));

  }

  @Override
  public void autonomousPeriodic() {
    EncoderSetter.updateEncoders();
    SmartDashboard.putNumber("distance", RobotMap.avgPositionInMeters);
    if (!RobotMap.reachedGoal) {
      if (RobotMap.avgPositionInMeters < 1.75) {
        RobotMap.m_drive.arcadeDrive(0.5, 0.0);
      } else {
        RobotMap.reachedGoal = true;
        RobotMap.m_drive.arcadeDrive(0.0, 0.0);
      }
    }

    if (RobotMap.reachedGoal & !RobotMap.shotFirstShotInAuto) {
      if (!RobotMap.inFiringCoroutine) {
        RobotMap.inFiringCoroutine = true;
        RobotMap.fullShooterPower = true;
        RobotMap.shotFirstShotInAuto = true;
        RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
        RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
        RobotMap.PewPewMotor1.config_kI(0, RobotMap.kI);
        RobotMap.PewPewMotor1.config_kD(0, RobotMap.kD);
        RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
        RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
        RobotMap.PewPewMotor2.config_kI(0, RobotMap.kI);
        RobotMap.PewPewMotor2.config_kD(0, RobotMap.kD);
        RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
      }
    }

    // double error = -RobotMap.gyro.getRate();
    /*
     * /*
     * // double error = -RobotMap.gyro.getRate();
     * 
     * /*
     * //https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/
     * encoders-software.html
     * //other side is flipped internally
     * if (RobotMap.avgPositionInMeters < 2.3) {
     * //face of intake direction is negative
     * drivetrain.arcadeDriveVoltage(-0.2,.5 - 1 * error, 0.75, -0.75);
     * } else {
     * drivetrain.arcadeDriveVoltage(0, .5 - 1 * error, 0.75, -0.75);
     * }
     * 
     * 
     * currentAutoTime = startAutoTime - System.currentTimeMillis();
     * 
     * if (currentAutoTime >= 3000) {
     * // RobotMap.pneumaticDoubleSolenoid.set(Value.kReverse);
     * } else if (currentAutoTime >= 3500) {
     * RobotMap.IntakeMotor1.set(VictorSPXControlMode.PercentOutput, 1);
     * } else if (currentAutoTime >= 4500) {
     * RobotMap.IntakeMotor1.set(VictorSPXControlMode.PercentOutput, 0);
     * } else if (currentAutoTime >= 5500) {
     * // RobotMap.pneumaticDoubleSolenoid.set(Value.kForward);
     * } else if (currentAutoTime >= 6200) {
     * if (currentAutoTime >= 11700) {
     * RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
     * RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0.0);
     * RobotMap.inFiringCoroutine = false;
     * } else if (currentAutoTime >= 11200) {
     * RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
     * } else if (currentAutoTime >= 9200) {
     * RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
     * } else if (currentAutoTime >= 8700) {
     * RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
     * } else {
     * RobotMap.PewPewMotor2.set(ControlMode.Velocity, RobotMap.velocityTarget);
     * }
     * }
     * 
     * }
     * 
     * public SequentialCommandGroup getAutonoumous(int num) {
     * // ITZ PATHWEAVER LAND from here on out
     * var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
     * new SimpleMotorFeedforward(
     * DriveSubsystem.DriveConstants.ksVolts,
     * DriveSubsystem.DriveConstants.kvVoltSecondsPerMeter,
     * DriveSubsystem.DriveConstants.kaVoltSecondsSquaredPerMeter),
     * DriveSubsystem.DriveConstants.kDriveKinematics,
     * 10);
     * 
     * TrajectoryConfig config = new TrajectoryConfig(
     * DriveSubsystem.DriveConstants.kMaxSpeedMetersPerSecond,
     * DriveSubsystem.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
     * // Add kinematics to ensure max speed is actually obeyed
     * .setKinematics(DriveSubsystem.DriveConstants.kDriveKinematics)
     * // Apply the voltage constraint
     * .addConstraint(autoVoltageConstraint);
     * 
     * RamseteCommand ramseteCommand = new RamseteCommand(
     * trajectory,
     * m_robotDrive::getPose,
     * new RamseteController(DriveSubsystem.DriveConstants.kRamseteB,
     * DriveSubsystem.DriveConstants.kRamseteZeta),
     * new SimpleMotorFeedforward(
     * DriveSubsystem.DriveConstants.ksVolts,
     * DriveSubsystem.DriveConstants.kvVoltSecondsPerMeter,
     * DriveSubsystem.DriveConstants.kaVoltSecondsSquaredPerMeter),
     * DriveSubsystem.DriveConstants.kDriveKinematics,
     * m_robotDrive::getWheelSpeeds,
     * new PIDController(DriveSubsystem.DriveConstants.kPDriveVel, 0, 0),
     * new PIDController(DriveSubsystem.DriveConstants.kPDriveVel, 0, 0),
     * // RamseteCommand passes volts to the callback
     * m_robotDrive::tankDriveVolts,
     * m_robotDrive);
     * 
     * // Reset odometry to the starting pose of the trajectory.
     * m_robotDrive.resetOdometry(trajectory.getInitialPose());
     * 
     * // Run path following command, then stop at the end.
     * return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
     */
  }

  @Override
  public void testPeriodic() {
  }

}