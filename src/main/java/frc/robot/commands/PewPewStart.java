package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PewPewStart extends Command {
  public PewPewStart() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }
  double velocity = 0.0;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.inFiringCoroutine = true;
    RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
    RobotMap.FeederMotor.config_kF(0, RobotMap.kPIndex);
    RobotMap.FeederMotor.config_kP(0, RobotMap.kFIndex);
    RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    velocity = RobotMap.fullShooterPower ? RobotMap.velocityFeeder : RobotMap.velocityTarget/2.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
      //delay is measured in milliseconds
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
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !RobotMap.inFiringCoroutine;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.pneumaticSingleSolenoid.set(false);
    RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    RobotMap.inFiringCoroutine = false;

    RobotMap.pneumaticSingleSolenoid.set(false);
    RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
  }

}// class PewPewStart