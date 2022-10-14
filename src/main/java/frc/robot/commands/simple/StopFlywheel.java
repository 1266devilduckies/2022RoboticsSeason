

package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class StopFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  double startTime;
  public StopFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("stopped flywheel");
    shooterSubsystem.setToCoast();
    startTime = Timer.getFPGATimestamp();
    //shooterSubsystem.setIndexerMotor(-1.0);
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return true;//(Timer.getFPGATimestamp() - startTime) >= Constants.indexingTimeMillis /1000.;
  }

  @Override
  public void end(boolean interrupted) {
    //shooterSubsystem.setIndexerMotor(0.0);
  }
}