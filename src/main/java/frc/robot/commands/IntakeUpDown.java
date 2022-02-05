package frc.robot.commands;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeUpDown extends Command {

  //this is wrong, they are double

  //WPI Docs check double solenoid section for info

  /*the double solenoind MUST have either a public, 
  protected or package-private (default) access modifier.*/
  DoubleSolenoid pneumaticDoubleSolenoid = new DoubleSolenoid(6,PneumaticsModuleType.CTREPCM, 1,2);
  private int count = 0;

  //WPI Docs check Pressure Transducers for info on check pressure (param 1 is module ID)
  //private final Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
  //check pressure with: double current = phCompressor.getPressure();

  //Solenoid pneumaticLeft = new Solenoid(PneumaticsModuleType.REVPH, 1);
  //Solenoid pneumaticRight = new Solenoid(PneumaticsModuleType.REVPH, 1);

  public IntakeUpDown() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    pneumaticDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void togglePiston() {
    pneumaticDoubleSolenoid.toggle();
    count++;
    SmartDashboard.putNumber("pressed", (double)count);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
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
