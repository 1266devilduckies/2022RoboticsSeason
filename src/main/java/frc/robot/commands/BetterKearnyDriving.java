package frc.robot.commands;

import frc.robot.JoystickController;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BetterKearnyDriving extends CommandBase {
  // Drivetrain drivetrain = Robot.drivetrain;
  JoystickController mainJoystick = JoystickController.MAIN_JOYSTICK;
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  double threshold = 0.05;

  public BetterKearnyDriving() {
    // requires(Robot.drivetrain);
  }

  @Override
  public void initialize() {
    RobotMap.MainLeftMotorBack.configOpenloopRamp(0);
      RobotMap.MainLeftMotorFront.configOpenloopRamp(0);
      RobotMap.MainRightMotorBack.configOpenloopRamp(0);
      RobotMap.MainRightMotorFront.configOpenloopRamp(0);
  }

  @Override
  public void execute() {
    if (!RobotMap.pilotDisabled) {
      double lVal = mainJoystick.getLeftStickY();
      double rVal = mainJoystick.getRightStickX();
      //was 0.5 for all

      if (Math.abs(lVal) < 0.05) {
        RobotMap.m_drive.curvatureDrive(0.0, rVal*0.8, true);
      } else {
        RobotMap.m_drive.curvatureDrive(-lVal*0.8, rVal *0.8, false); //*.5
      }
    }
  }

  @Override
  public void end(boolean interuppted) {
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}