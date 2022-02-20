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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.commands.PewPewStart;
import frc.robot.Util;
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

public class Robot extends TimedRobot{
public static Drivetrain drivetrain;  
public static Intake intake;
public static Shooter shooter;
public static double turnY;
public static double moveX;

//CALIBRATE VALUE TO OUR ROBOT LATER
public static final double ksVolts = 0.22;
public static final double kvVoltSecondsPerMeter = 1.98;
public static final double kaVoltSecondsSquaredPerMeter = 0.2;
public static final double kPDriveVel = 8.5;
public static final double kTrackwidthMeters = 0.71;
public static final double kMaxSpeedMetersPerSecond = 3;
public static final double kMaxAccelerationMetersPerSecondSquared = 3;
public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

public static final double kRamseteB = 2;
public static final double kRamseteZeta = 0.7;

Command m_autonomousCommand;
SendableChooser<Command> m_chooser = new SendableChooser<>();
@Override
public void robotInit(){
  RobotMap.init();
  drivetrain = new Drivetrain();
  Util.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorBack);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorFront);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorBack);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorFront);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor1);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor2);
  Util.setEncoderDefaultPhoenixSettings(RobotMap.FeederMotor);
  RobotMap.PewPewMotor2.setInverted(true);
  RobotMap.PewPewMotor1.setInverted(false);
  
  //configure the PID
  
  RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
  RobotMap.PewPewMotor1.config_kP(0, RobotMap.kF/5);
  RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
  RobotMap.PewPewMotor2.config_kP(0, RobotMap.kF/5);
  //

  RobotMap.gyro.calibrate();
  intake = new Intake();
  shooter = new Shooter();
  JoystickController.Init();
  SmartDashboard.putData("Auto mode", m_chooser);
  String trajectoryJSON = "paths/start1.wpilib.json";
  
Trajectory trajectory = new Trajectory();
  try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
 } catch (IOException e) {
   // 
   e.printStackTrace();
}
}
@Override
public void robotPeriodic(){
  
}
@Override
public void disabledInit(){
}
@Override
public void disabledPeriodic(){
}
@Override
public void autonomousInit(){

}


@Override
public void teleopInit(){
  RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0);
  RobotMap.MainLeftMotorFront.setSelectedSensorPosition(0);
  RobotMap.MainRightMotorBack.setSelectedSensorPosition(0);
  RobotMap.MainRightMotorFront.setSelectedSensorPosition(0);
  RobotMap.FeederMotor.setSelectedSensorPosition(0);
  drivetrain.arcadeDriveVoltage(0.,0., 0.75, -0.75);
  Scheduler.getInstance().add(new BetterKearnyDriving());
}

@Override
public void teleopPeriodic(){
  //periodic events
  JoystickController.checkForPneumatics();
  limeLightDataFetcher.fetchData();
  
  //logging data
  SmartDashboard.putBoolean("in coroutine", RobotMap.inFiringCoroutine);
  SmartDashboard.putNumber("gyro rotation", RobotMap.gyro.getAngle());
  SmartDashboard.putNumber("diffrence x", limeLightDataFetcher.getdegRotationToTarget());
  SmartDashboard.putNumber("difference y", limeLightDataFetcher.getdegVerticalToTarget());

  Scheduler.getInstance().run();
}

@Override
public void autonomousPeriodic(){
  double error = -RobotMap.gyro.getRate();

  //https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/encoders-software.html
  //other side is flipped internally
  if (RobotMap.avgPositionInMeters < 2.3) {
    //face of intake direction is negative
    drivetrain.arcadeDriveVoltage(-0.2,.5 - 1 * error, 0.75, -0.75);
  } else {
    drivetrain.arcadeDriveVoltage(0, .5 - 1 * error, 0.75, -0.75);
  }

  //ITZ PATHWEAVER LAND from here on out
  var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
      ksVolts,
      kvVoltSecondsPerMeter,
      kaVoltSecondsSquaredPerMeter),
      kDriveKinematics,
      10);

  TrajectoryConfig config =
    new TrajectoryConfig(
      kMaxSpeedMetersPerSecond,
      kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

  Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

  /*RamseteCommand ramseteCommand =
    new RamseteCommand(
      exampleTrajectory,
      //getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(
      ksVolts,
      kvVoltSecondsPerMeter,
      kaVoltSecondsSquaredPerMeter),
      kDriveKinematics,
      //getWheelSpeeds,
      new PIDController(kPDriveVel, 0, 0),
      new PIDController(kPDriveVel, 0, 0)
      // RamseteCommand passes volts to the callback
      //drivetrain::tankDriveVoltage,
      //drivetrain
      );*/

  // Reset odometry to the starting pose of the trajectory.
  //resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

}

@Override
public void testPeriodic(){  
}

}
