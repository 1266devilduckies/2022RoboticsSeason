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
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (RobotMap.checkerBoardForShooter % 2 == 0) {
    if (Math.abs(RobotMap.dvm1) < 50 & Math.abs(RobotMap.dvm2) < 50 & !RobotMap.releasingBall) {
      //feed into shooter
      RobotMap.timeSinceStartedBeingReleased = System.currentTimeMillis();
      RobotMap.releasingBall = true;
    }
    if (Math.abs(RobotMap.dvm1) > 0.001) {
      RobotMap.PewPewMotor1VelocityEstimate += Math.signum(RobotMap.dvm1) * 0.0005;
    }
    if (Math.abs(RobotMap.dvm2) > 0.001) {
      RobotMap.PewPewMotor2VelocityEstimate += Math.signum(RobotMap.dvm2) * 0.0005;
    }
    RobotMap.PewPewMotor1.set(ControlMode.PercentOutput, RobotMap.PewPewMotor1VelocityEstimate);
    RobotMap.PewPewMotor2.set(ControlMode.PercentOutput, RobotMap.PewPewMotor2VelocityEstimate);
  }
  SmartDashboard.putNumber("dvm1", Math.abs(RobotMap.dvm1));
    SmartDashboard.putBoolean("shooting", RobotMap.releasingBall);
    SmartDashboard.putNumber("motor velocity estimate", RobotMap.PewPewMotor1VelocityEstimate);
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