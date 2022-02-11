package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class PewPewStart extends Command {//--------------class--------------
  final private double targetRPM = 1000.0;
  final private double bound = 1.0;
  public PewPewStart() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  private void releaseBall() {
    //moveBallMotor();
    if (RobotMap.PewPewMotor1RPM < (targetRPM - bound) & RobotMap.PewPewMotor1RPM > (targetRPM + bound)) {
      //stopBallMotor();
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!RobotMap.inSubroutine) {
      RobotMap.inSubroutine = true; 
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  
  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  
  }//class PewPewStart