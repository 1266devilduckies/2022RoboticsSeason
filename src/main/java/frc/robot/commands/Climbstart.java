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
import frc.robot.Robot;
import frc.robot.RobotMap;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climbstart extends CommandBase {
  public Climbstart() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    RobotMap.Climber1.set(ControlMode.PercentOutput, 1.0);
    RobotMap.Climber2.set(ControlMode.PercentOutput, 1.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

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
