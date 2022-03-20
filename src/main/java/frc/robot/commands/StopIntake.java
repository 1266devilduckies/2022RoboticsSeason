package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class StopIntake extends CommandBase {
  public StopIntake() {
    // Use requires() here to declare subsystem dependencies
    //addRequirements(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("reversed", true);
    RobotMap.IntakeMotor1.set(ControlMode.PercentOutput, 0.0);
    RobotMap.pneumaticSingleSolenoid.set(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}// class
