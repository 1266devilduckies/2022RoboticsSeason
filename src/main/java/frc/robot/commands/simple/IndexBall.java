package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class IndexBall extends CommandBase {
  Shooter shooterSubsystem;
  double startTime;
  public IndexBall(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shooterSubsystem.setIndexerMotor(1.0);
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) >= Constants.indexingTimeMillis /1000.;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setIndexerMotor(0.0); //dont check if it finished the logic as this acts as a watchdog
  }
}