package frc.robot.commands;

import frc.robot.JoystickController;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotMap;
//import javax.management.modelmbean.RequiredModelMBean;
import frc.robot.EncoderSetter;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  }

  @Override
  public void execute() {
    // ps4 & xbox apis have it so that going forward on the y makes it go negative

    double lVal = mainJoystick.getLeftStickY();
    double rVal = mainJoystick.getRightStickX();
    /*
     * if (Math.abs(lVal) < threshold) {
     * RobotMap.MainLeftMotorFront.configOpenloopRamp(0.0);
     * RobotMap.MainLeftMotorBack.configOpenloopRamp(0.0);
     * RobotMap.MainRightMotorFront.configOpenloopRamp(0.0);
     * RobotMap.MainRightMotorBack.configOpenloopRamp(0.0);
     * SmartDashboard.putNumber("left wheel speeds",
     * RobotMap.m_drive.tankDriveIK(rVal, -rVal, false).left
     * - RobotMap.m_drive.tankDriveIK(rVal, -rVal, false).right);
     * RobotMap.m_drive.tankDrive(rVal * 0.8 - RobotMap.tankDriveInPlaceError, -rVal
     * * 0.8, false);
     * } else {
     * RobotMap.MainLeftMotorFront.configOpenloopRamp(0.5);
     * RobotMap.MainLeftMotorBack.configOpenloopRamp(0.5);
     * RobotMap.MainRightMotorFront.configOpenloopRamp(0.5);
     * RobotMap.MainRightMotorBack.configOpenloopRamp(0.5);
     * SmartDashboard.putNumber("working working working", 1.0);
     * RobotMap.m_drive.arcadeDrive(-mainJoystick.getLeftStickY(),
     * mainJoystick.getRightStickX(), false);
     * }
     */
     RobotMap.MainLeftMotorBack.configOpenloopRamp(0.2);
     RobotMap.MainLeftMotorFront.configOpenloopRamp(0.2);
     RobotMap.MainRightMotorBack.configOpenloopRamp(0.2);
     RobotMap.MainRightMotorFront.configOpenloopRamp(0.2);
     
     double normSpeed = 0.65;
     double errL = 0.0;
     double errR = 0.0;
    if (mainJoystick.getLeftTrigger() && mainJoystick.getRightTrigger()) {
    } else if (mainJoystick.getLeftTrigger()) {
       RobotMap.m_drive.tankDrive(-normSpeed + errL, normSpeed + errR);
     } else if (mainJoystick.getRightTrigger()) {
      RobotMap.m_drive.tankDrive(normSpeed + errL, -normSpeed + errR);
     } else {
     if (Math.abs(lVal) < 0.05) {
      RobotMap.m_drive.curvatureDrive(0.0, rVal, true);
     } else {
       RobotMap.m_drive.curvatureDrive(-lVal, rVal*0.5,false);
     }
    }
    EncoderSetter.updateEncoders();
  }

  @Override
  public void end(boolean interuppted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}