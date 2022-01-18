package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import edu.wpi.first.wpilibj.drive.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.*;

import java.io.IOException;
import java.nio.file.Path;

//This is basically our main class, we just don't use Main.java for clarity (i guess) -JM

public class Robot extends TimedRobot{
public static Drivetrain drivetrain;  
public static Intake intake;
public static Shooter shooter;

Command m_autonomousCommand;
SendableChooser<Command> m_chooser = new SendableChooser<>();
@Override
public void robotInit(){
  RobotMap.init();
  drivetrain = new Drivetrain();
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
  Auto auto = new Auto(2.0);
  auto.initialize(); //creates the start time for the dt check
  Scheduler.getInstance().add(auto);
}

@Override
public void teleopInit(){
   Scheduler.getInstance().add(new BetterKearnyDriving());
   
}
@Override
public void teleopPeriodic(){
  Scheduler.getInstance().run();
}

@Override
public void autonomousPeriodic(){
   Scheduler.getInstance().run();
}

@Override
public void testPeriodic(){  
}
}
