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
  double startTime;
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    RobotMap.inFiringCoroutine = true;
    if(fullShooterPower){
      velocity = RobotMap.velocityTarget;
    } else{
      velocity = RobotMap.velocityTarget / 2.0;
    }
    RobotMap.PewPewMotor1.setSelectedSensorPosition(0);
    RobotMap.PewPewMotor2.setSelectedSensorPosition(0);
    RobotMap.FeederMotor.config_kP(0, RobotMap.kPIndex);
    RobotMap.FeederMotor.config_kF(0, RobotMap.kFIndex);
    RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
    RobotMap.velocityCurrent = velocity;
    RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
    RobotMap.PewPewMotor1.set(ControlMode.Velocity, velocity);
    RobotMap.PewPewMotor2.set(ControlMode.Velocity, velocity);
    RobotMap.pneumaticSingleSolenoid.set(true);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= 500 && 
    (((RobotMap.PewPewMotor2.getSelectedSensorVelocity() +100 ) - velocity) >= 0) && 
    (((RobotMap.PewPewMotor1.getSelectedSensorVelocity() +100 ) - velocity) >= 0); //have it activate after it reaches it or passes it so that theres more speed than intended
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    
    if (interrupted) {
      RobotMap.inFiringCoroutine = false;
      RobotMap.isOneBall = false;
      RobotMap.pneumaticSingleSolenoid.set(false);
      RobotMap.overrideVelocity = -1.0;
      RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
      RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
      RobotMap.PewPewMotor1.set(ControlMode.Velocity, 0);
    }
  }

}// class PewPewStart