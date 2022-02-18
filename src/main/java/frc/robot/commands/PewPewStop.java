package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class PewPewStop extends Command {//--------------class--------------
  public static int released = 0;
  public PewPewStop() {
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
    //motor code to stop for when build implements their design
    //
    //motor code to stop for when build implements their design
    RobotMap.timeSinceStartedBeingReleased = -1;
    RobotMap.releasingBall = false;
    //RobotMap.PewPewMotor2.set(ControlMode.PercentOutput, 0.0);

    //RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0.0);
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
  
  }//class PewPewStop
