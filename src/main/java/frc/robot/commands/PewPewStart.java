package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PewPewStart extends CommandBase {
  double velocity = -1.0;
  public PewPewStart(boolean slowShot) {
    RobotMap.fullShooterPower = !slowShot;
  }
  public PewPewStart(boolean slowShot, boolean oneBall) {
    RobotMap.fullShooterPower = !slowShot;
    RobotMap.isOneBall = oneBall;
  }
  public PewPewStart(boolean slowShot, double overrideVelocity) {
    RobotMap.fullShooterPower = !slowShot;
    velocity = overrideVelocity;
  }


  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.inFiringCoroutine = true;
    RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    if (velocity < 0) {
    velocity = RobotMap.fullShooterPower ? RobotMap.velocityFeeder : RobotMap.velocityTarget / 2.0;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
    // delay is measured in milliseconds
    if (!RobotMap.isOneBall) {
    if (dt >= 4500) {
      RobotMap.inFiringCoroutine = false;
    } else if (dt >= 3000) {
      RobotMap.FeederMotor.set(ControlMode.Velocity, RobotMap.velocityFeeder);
    } else if (dt >= 2000) {
      RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    } else if (dt >= 1550) {
      RobotMap.FeederMotor.set(ControlMode.Velocity, RobotMap.velocityFeeder);
    } else {
      RobotMap.pneumaticSingleSolenoid.set(true);
      RobotMap.PewPewMotor2.set(ControlMode.Velocity, velocity);
    }
  } else {
    if (dt >= 1000) {
      RobotMap.inFiringCoroutine = false;
    } else if (dt >= 500) {
      RobotMap.FeederMotor.set(ControlMode.Velocity, RobotMap.velocityFeeder);
    } else {
      RobotMap.PewPewMotor2.set(ControlMode.Velocity, velocity);
    }
  }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !RobotMap.inFiringCoroutine || (RobotMap.overrideVelocity > 0 & RobotMap.limeLightDistance < 0);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotMap.inFiringCoroutine = false;
    RobotMap.fullShooterPower = true;
    RobotMap.isOneBall = false;
    RobotMap.pneumaticSingleSolenoid.set(false);
    RobotMap.overrideVelocity = -1.0;
    RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
  }

}// class PewPewStart