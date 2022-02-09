package frc.robot;

import static edu.wpi.first.wpilibj.CounterBase.EncodingType.k1X;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
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
//import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
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
public static Encoder leftEncoder;
public static Encoder rightEncoder;

Command m_autonomousCommand;
SendableChooser<Command> m_chooser = new SendableChooser<>();
@Override
public void robotInit(){
  RobotMap.init();
  drivetrain = new Drivetrain();
  Util.setEncodersDefaultPhoenixSettings();
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
  drivetrain.arcadeDriveVoltage(0.,0., 0.75, -0.75);
  Scheduler.getInstance().add(new BetterKearnyDriving());
}
@Override
public void teleopPeriodic(){
  JoystickController.checkForPneumatics();
  limeLightDataFetcher.fetchData();
  SmartDashboard.putNumber("gyro rotation", RobotMap.gyro.getAngle());
  SmartDashboard.putNumber("diffrence x", limeLightDataFetcher.getdegRotationToTarget());
  SmartDashboard.putNumber("difference y", limeLightDataFetcher.getdegVerticalToTarget());
  Scheduler.getInstance().run();
}

@Override
public void autonomousPeriodic(){
  double error = -RobotMap.gyro.getRate();
  Util.updateEncoders();
  //https://docs.wpilib.org/en/latest/docs/software/hardware-apis/sensors/encoders-software.html
  //other side is flipped internally
  if (RobotMap.avgPositionInMeters < 2.3) {
    //face of intake direction is negative
    drivetrain.arcadeDriveVoltage(-0.2,.5 - 1 * error, 0.75, -0.75);
  } else {
    drivetrain.arcadeDriveVoltage(0, .5 - 1 * error, 0.75, -0.75);
  }
}

@Override
public void testPeriodic(){  
}

}
