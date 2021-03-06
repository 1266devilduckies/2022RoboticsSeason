package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class PewPewStart extends CommandBase {
  boolean fullShooterPower;
  double velocity = -1.0;

  public PewPewStart(boolean slowShot) {
    fullShooterPower = !slowShot;
  }

  public PewPewStart(boolean slowShot, boolean oneBall) {
    fullShooterPower = !slowShot;
    RobotMap.isOneBall = oneBall;
  }

  public PewPewStart(boolean slowShot, boolean oneBall, double overrideVelocity) {
    fullShooterPower = !slowShot;
    RobotMap.isOneBall = oneBall;
    velocity = overrideVelocity;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.inFiringCoroutine = true;
    if(fullShooterPower){
      velocity = RobotMap.velocityTarget;
      RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
      RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
      
    } else{
      RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP2);
      RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF2);
      RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP2);
      RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF2);
      velocity = RobotMap.velocityTarget / 2.0;
    }
    RobotMap.FeederMotor.config_kP(0, RobotMap.kPIndex);
    RobotMap.FeederMotor.config_kF(0, RobotMap.kFIndex);
    RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
    // delay is measured in milliseconds
    if (!RobotMap.isOneBall) {
      if (dt >= 3000) {
        RobotMap.inFiringCoroutine = false;
      } else if (dt >= 2000) {
        RobotMap.FeederMotor.set(ControlMode.Velocity, RobotMap.velocityFeeder);
      } else if (dt >= 1000) {
        RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
      } else if (dt >= 500) {
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
    return !RobotMap.inFiringCoroutine;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotMap.inFiringCoroutine = false;
    RobotMap.isOneBall = false;
    RobotMap.pneumaticSingleSolenoid.set(false);
    RobotMap.overrideVelocity = -1.0;
    RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
  }

}// class PewPewStart