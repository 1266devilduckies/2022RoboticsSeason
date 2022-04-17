package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StopFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  public StopFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setTargetRPM(0);
    shooterSubsystem.setToCoast();
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}