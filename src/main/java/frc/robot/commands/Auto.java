/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//This is Auto.java. It runs the stuff for the automated phase -JM


package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
//import frc.robot.RobotMap;

public class Auto extends Command {
  private double timeout, startTime, currentTime;
  public Auto(double timeout) {
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);  
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    //run commands
    Robot.drivetrain.arcadeDriveVoltage(0.3, 0, 1, 0);
    //
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //update the time difference
    currentTime = Timer.getFPGATimestamp();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //Think outside the box
    return timeout < (currentTime - startTime);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.arcadeDriveVoltage(0, 0, 0, 0); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.drivetrain.arcadeDriveVoltage(0, 0, 0, 0);
  }
}
