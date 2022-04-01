package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoystickController;
import frc.robot.RobotMap;

public class Wait extends CommandBase {
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;

  double cooldown;
  public Wait(double initCooldown) {
    // Use requires() here to declare subsystem dependencies
    cooldown = initCooldown;
  }

  // Called just before this Command runs the first time
  double startTime;
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - startTime) >= cooldown;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }

}// class
