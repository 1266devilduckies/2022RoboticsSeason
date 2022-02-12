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
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //velocity in ticks per 100ms divided by 3 cause we want a third of it
    double vel = RobotMap.PewPewMotor1.getSelectedSensorVelocity(0);
    
    //RobotMap.velocityTarget --> target velocity
    //RobotMap.velocityThreshold --> how far away from the target velocity we are okay with going
    double PewPewDeltaV = RobotMap.velocityTarget - vel; //DeltaV is the current speed's distance from the target speed
    if (Math.abs(PewPewDeltaV) > 0.0001) {
      RobotMap.PewPewMotor1VelocityEstimate += Math.signum(PewPewDeltaV) * 0.001;//just look up what signum (aka sign) does
    }
    SmartDashboard.putNumber("motor velocity estimate", RobotMap.PewPewMotor1VelocityEstimate);
    RobotMap.PewPewMotor1.set(ControlMode.PercentOutput, RobotMap.PewPewMotor1VelocityEstimate);
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