package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickController;
import frc.robot.RobotMap;

public class ClimberDown extends CommandBase {

  public ClimberDown() {
    // Use requires() here to declare subsystem dependencies
    
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.Climber1.config_kP(0, 0.02);
    RobotMap.Climber1.configForwardSoftLimitThreshold(0, 0);
    RobotMap.Climber1.configReverseSoftLimitThreshold(-RobotMap.upperBoundClimber, 0);
    RobotMap.Climber1.configForwardSoftLimitEnable(true, 0);
    RobotMap.Climber1.configReverseSoftLimitEnable(true, 0);
    RobotMap.Climber2.config_kP(0, 0.02);
    RobotMap.Climber2.configForwardSoftLimitThreshold(0, 0);
    RobotMap.Climber2.configReverseSoftLimitThreshold(-RobotMap.upperBoundClimber, 0);
    RobotMap.Climber2.configForwardSoftLimitEnable(true, 0);
    RobotMap.Climber2.configReverseSoftLimitEnable(true, 0);

    RobotMap.Climber1.set(ControlMode.Position, 0);
    RobotMap.Climber2.set(ControlMode.Position, 0);
    RobotMap.operatorIsControlling = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotMap.operatorIsControlling = true;
  }

}// class
