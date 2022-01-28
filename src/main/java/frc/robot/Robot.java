package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auto;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.subsystems.Drivetrain;
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

Command m_autonomousCommand;
SendableChooser<Command> m_chooser = new SendableChooser<>();
@Override
public void robotInit(){
  RobotMap.init();
  drivetrain = new Drivetrain();
  intake = new Intake();
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
<<<<<<< Updated upstream
  Scheduler.getInstance().add(auto);
}
=======
  Scheduler.getInstance().add(auto);*/
  encoder.setDistancePerPulse(1./256.);
 
  }

>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
   Scheduler.getInstance().run();
=======
  Scheduler.getInstance().run();
  if(encoder.getDistance() < 5) {
    //Drivetrain.tankDrive(0.5, 0.5);
    Robot.drivetrain.arcadeDriveVoltage(0.5, 0.5,0.75,-0.75);
} else {
  Robot.drivetrain.arcadeDriveVoltage(0.0, 0.0,0.75,-0.75);
    //Drivetrain.tankDrive(0, 0);
} 
>>>>>>> Stashed changes
}

@Override
public void testPeriodic(){  
}
}
