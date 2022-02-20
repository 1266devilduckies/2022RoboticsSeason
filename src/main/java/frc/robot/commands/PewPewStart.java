package frc.robot.commands;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
public class PewPewStart extends Command {//--------------class--------------
  public PewPewStart() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (!RobotMap.inFiringCoroutine) {
      RobotMap.inFiringCoroutine = true;
      RobotMap.timeSinceStartedBeingReleased = System.currentTimeMillis();
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (RobotMap.inFiringCoroutine) {
      long difference = (System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleased);
      SmartDashboard.putNumber("difference", difference);
      SmartDashboard.putNumber("current time", System.currentTimeMillis());
      SmartDashboard.putNumber("past time", RobotMap.timeSinceStartedBeingReleased);
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleased) >= 4000) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0.0);
        RobotMap.inFiringCoroutine = false;
        return;
      }
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleased) >= 3000) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
        return;
      }
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleased) >= 2000) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 0.0);
        return;
      }
      if ((System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleased) >= 1000) {
        RobotMap.FeederMotor.set(ControlMode.PercentOutput, 1.0);
        return;
      } else {
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, RobotMap.velocityTarget);
        RobotMap.inFiringCoroutine = false;
      }
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