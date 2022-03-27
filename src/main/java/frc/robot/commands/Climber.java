package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickController;
import frc.robot.RobotMap;

public class Climber extends CommandBase {
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  public Climber() {
    // Use requires() here to declare subsystem dependencies
    
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.Climber1.config_kP(0, 0.02);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double lVal = -coPilotJoystick.getLeftStickY();
    double ticks = -RobotMap.Climber1.getSelectedSensorPosition();
    double direction = Math.signum(lVal);
    if (RobotMap.operatorIsControlling) {
    if (ticks > RobotMap.upperBoundClimber /*|| ticks2 > RobotMap.upperBoundClimber*/) {
      RobotMap.climberFlag = 1;
      
    } else if (ticks < RobotMap.lowerBoundClimber /*|| ticks2 > RobotMap.lowerBoundClimber*/) {
      RobotMap.climberFlag = -1;
    } else {
      RobotMap.climberFlag = 0;
    }
    if (RobotMap.climberFlag == 1) {
      if (direction < 0) {
      RobotMap.Climber1.set(ControlMode.PercentOutput, -1*-RobotMap.climberSpeed);
      } else {
        RobotMap.Climber1.set(ControlMode.PercentOutput, 0.0);
      }
    } else if (RobotMap.climberFlag == -1) {
      if (direction > 0) {
      RobotMap.Climber1.set(ControlMode.PercentOutput, -1*RobotMap.climberSpeed);
      } else {
        RobotMap.Climber1.set(ControlMode.PercentOutput, 0.0);
      }
    } else {
      RobotMap.Climber1.set(ControlMode.PercentOutput, -1*direction * RobotMap.climberSpeed);
    }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    
  }

}// class
