package frc.robot.commands;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
public class PewPewStart extends Command {//--------------class--------------
<<<<<<< HEAD
  final private double targetRPM = 1000.0;
  final private double bound = 1.0;
=======
  public static boolean releasingBall = false;
  public static long timeSinceStartedBeingReleased;
>>>>>>> vision
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
    //delta v motor 1
    double dvm1 = RobotMap.velocityTarget - RobotMap.PewPewMotor1.getSelectedSensorVelocity(0);
    //delta v motor 2
    double dvm2 = RobotMap.velocityTarget - RobotMap.PewPewMotor2.getSelectedSensorVelocity(0);
    if (Math.abs(dvm1) < 10 & Math.abs(dvm2) < 10 & !releasingBall) {
      //feed into shooter
      timeSinceStartedBeingReleased = System.currentTimeMillis();
      releasingBall = true;
    }
    if (Math.abs(dvm1) > 0.001) {
      RobotMap.PewPewMotor1VelocityEstimate += Math.signum(dvm1) * 0.0005;
    }
    if (Math.abs(dvm2) > 0.001) {
      RobotMap.PewPewMotor2VelocityEstimate += Math.signum(dvm2) * 0.0005;
    }
    RobotMap.PewPewMotor1.set(ControlMode.PercentOutput, RobotMap.PewPewMotor1VelocityEstimate);
    RobotMap.PewPewMotor2.set(ControlMode.PercentOutput, RobotMap.PewPewMotor2VelocityEstimate);
    SmartDashboard.putBoolean("shooting", releasingBall);
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