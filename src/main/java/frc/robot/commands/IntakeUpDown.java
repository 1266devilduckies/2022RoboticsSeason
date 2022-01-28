package frc.robot.commands;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
<<<<<<< HEAD

public class IntakeUpDown extends Command {

  Solenoid pneumaticLeft = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Solenoid pneumaticRight = new Solenoid(PneumaticsModuleType.REVPH, 1);

=======
 
public class IntakeUpDown extends Command {
 
  Solenoid pneumaticLeft = new Solenoid(PneumaticsModuleType.REVPH, 1);
  Solenoid pneumaticRight = new Solenoid(PneumaticsModuleType.REVPH, 1);
 
>>>>>>> main
  public IntakeUpDown() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
  }
<<<<<<< HEAD

=======
 
>>>>>>> main
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
<<<<<<< HEAD

=======
 
>>>>>>> main
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pneumaticLeft.toggle();
    pneumaticRight.toggle();
  }
<<<<<<< HEAD

=======
 
>>>>>>> main
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
<<<<<<< HEAD
  
=======
 
>>>>>>> main
  // Called once after isFinished returns true
  @Override
  protected void end() {
  }
<<<<<<< HEAD

=======
 
>>>>>>> main
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
<<<<<<< HEAD
  
}//class
=======
 
}//class
 
>>>>>>> main
