package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickController;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climbstart extends CommandBase {
  long currentTimeClimber;
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  public Climbstart() {
    
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    currentTimeClimber = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      double dt = System.currentTimeMillis() - currentTimeClimber;
      currentTimeClimber = System.currentTimeMillis();
      double lVal = -coPilotJoystick.getLeftStickY();
      if (RobotMap.climberOverallTime < RobotMap.climberUpperBoundTime || RobotMap.climberLowerBoundTime >= 0.0) {
        if (Math.abs(lVal) < 0.05) {
          RobotMap.Climber1.set(ControlMode.PercentOutput, 0.0);
        } else if (lVal > 0.0) {
          RobotMap.climberOverallTime += dt;
          if (RobotMap.climberOverallTime < RobotMap.climberUpperBoundTime) {
          RobotMap.Climber1.set(ControlMode.PercentOutput, .75);
          }
        } else if (lVal < 0.0) {
          RobotMap.climberOverallTime -= dt;
          if (RobotMap.climberOverallTime > RobotMap.climberLowerBoundTime) {
          RobotMap.Climber1.set(ControlMode.PercentOutput, -.75);
          }
        }
      } else {
        RobotMap.Climber1.set(ControlMode.PercentOutput, 0.0);
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
