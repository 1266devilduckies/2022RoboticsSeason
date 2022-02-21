package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeUpDown extends Command {
  public IntakeUpDown() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.pneumaticDoubleSolenoid.toggle();
    //logic assumes kForward means extended, ready to eat balls
    if (RobotMap.pneumaticDoubleSolenoid.get() == Value.kForward) {
      SmartDashboard.putNumber("piston value", 0.0);
      RobotMap.timeSinceStartedBeingReleasedForSolenoids = System.currentTimeMillis();
    } else if (RobotMap.pneumaticDoubleSolenoid.get() == Value.kReverse) {
      SmartDashboard.putNumber("piston value", 1.0);
      RobotMap.timeSinceStartedBeingReleasedForSolenoids = -1;
      //RobotMap.IntakeMotor1.set(ControlMode.PercentOutput, 0.0);
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*
    if (RobotMap.timeSinceStartedBeingReleasedForSolenoids != -1 & (System.currentTimeMillis() - RobotMap.timeSinceStartedBeingReleasedForSolenoids) >= 1000) {
      RobotMap.IntakeMotor1.set(ControlMode.PercentOutput, 1.0);
    }*/
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  
  // Called once after isFinished returns true
  @Override
  protected void end() {

  }
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  
}//class
