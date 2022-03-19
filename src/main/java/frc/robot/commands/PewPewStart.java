package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PewPewStart extends CommandBase {
  public PewPewStart(boolean slowShot) {
    RobotMap.fullShooterPower = !slowShot;
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.shooter);
  }

  double velocity = 0.0;

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.inFiringCoroutine = true;
    RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
    RobotMap.FeederMotor.config_kF(0, RobotMap.kPIndex);
    RobotMap.FeederMotor.config_kP(0, RobotMap.kFIndex);
    RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    velocity = RobotMap.fullShooterPower ? RobotMap.velocityFeeder : RobotMap.velocityTarget / 2.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    long dt = System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForShooter;
    // delay is measured in milliseconds
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
  public boolean isFinished() {
    return !RobotMap.inFiringCoroutine;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotMap.inFiringCoroutine = false;
    RobotMap.fullShooterPower = true;
    RobotMap.pneumaticSingleSolenoid.set(false);
    RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
  }

}// class PewPewStart