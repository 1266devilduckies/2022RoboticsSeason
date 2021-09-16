/*----------------------------------------------------------------------------*/
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Auto extends Command {
  private double timeout, startTime, currentTime;
  public Auto(double timeout) {
    this.timeout = timeout;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drivetrain.arcadeDriveVoltage(0.3, 0, 1, 0);
    currentTime = Timer.getFPGATimestamp();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (timeout < (currentTime - startTime)) {
      return true;
      
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.arcadeDriveVoltage(0, 0, 0, 0); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.drivetrain.arcadeDriveVoltage(0, 0, 0, 0);
  }
}
