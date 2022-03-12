package frc.robot.commands;

import frc.robot.JoystickController;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotMap;
//import javax.management.modelmbean.RequiredModelMBean;
import frc.robot.EncoderSetter;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterKearnyDriving extends Command {
  // Drivetrain drivetrain = Robot.drivetrain;
  JoystickController mainJoystick = JoystickController.MAIN_JOYSTICK;
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  double threshold = 0.05;

  public BetterKearnyDriving() {
    // requires(Robot.drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // ps4 & xbox apis have it so that going forward on the y makes it go negative

    double lVal = mainJoystick.getLeftStickY();
    double rVal = mainJoystick.getRightStickX();
    if (Math.abs(lVal) < threshold) {
      SmartDashboard.putNumber("left wheel speeds", RobotMap.m_drive.tankDriveIK(rVal, -rVal, false).left
          - RobotMap.m_drive.tankDriveIK(rVal, -rVal, false).right);
      RobotMap.m_drive.tankDrive(rVal * 0.8 - RobotMap.tankDriveInPlaceError, -rVal * 0.8, false);
    } else {
      SmartDashboard.putNumber("working working working", 1.0);
      RobotMap.m_drive.arcadeDrive(-mainJoystick.getLeftStickY(), mainJoystick.getRightStickX(), false);
    }
    EncoderSetter.updateEncoders();
  }

  @Override
  protected void end() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}