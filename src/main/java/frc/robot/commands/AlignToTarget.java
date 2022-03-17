package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.limeLightDataFetcher;

public class AlignToTarget extends CommandBase {
  double curSpeed = 0.0;
  double calibratedAngle = RobotMap.angle + RobotMap.gyro.getAngle();

  public AlignToTarget() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    calibratedAngle = RobotMap.angle + RobotMap.gyro.getAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (RobotMap.angleMode) {
      curSpeed = RobotMap.alignerController.calculate(RobotMap.gyro.getAngle(), calibratedAngle);
      if (Math.abs(calibratedAngle - RobotMap.gyro.getAngle()) < 2.0) { // hacky way of damping
        RobotMap.m_drive.tankDrive(curSpeed, -curSpeed, false);
      }
    } else {
      if (limeLightDataFetcher.seeIfTargetsExist() == 1.0) {
        double pidOutput = RobotMap.alignerController.calculate(limeLightDataFetcher.getdegRotationToTarget(), 0.0);
        RobotMap.m_drive.tankDrive(-pidOutput, pidOutput, false);
      } else {
        RobotMap.m_drive.tankDrive(0.0, 0.0);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    RobotMap.angle = 0;
    calibratedAngle = RobotMap.gyro.getAngle(); // empty product is 0
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }

}// class
