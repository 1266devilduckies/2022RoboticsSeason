package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickController;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends CommandBase {
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  
  public Climber() {
    // Use requires() here to declare subsystem dependencies
    
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double lVal = -coPilotJoystick.getLeftStickY();

    if (RobotMap.climberFlag1 == 0 && RobotMap.climberFlag2 == 0 && Math.abs(lVal) > 0.5) {
    RobotMap.Climber1.set(ControlMode.PercentOutput, Math.signum(lVal) * RobotMap.climberSpeed);
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
