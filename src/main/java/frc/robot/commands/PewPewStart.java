package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PewPewStart extends Command {
  public PewPewStart() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.shooter);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (!RobotMap.inFiringCoroutine) {
      RobotMap.inFiringCoroutine = true;
      RobotMap.fullShooterPower = true;
      RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
      RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor1.config_kI(0, 0.0);
      RobotMap.PewPewMotor1.config_kD(0, 0.0);
      RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
      RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor2.config_kI(0, 0.0);
      RobotMap.PewPewMotor2.config_kD(0, 0.0);
      RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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

}// class PewPewStart