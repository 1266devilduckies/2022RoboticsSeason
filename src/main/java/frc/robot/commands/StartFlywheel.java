package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  public StartFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setTargetRPM(Constants.flywheelRPM);
    shooterSubsystem.setRPM(shooterSubsystem.getTargetRPM());
    shooterSubsystem.setIndexerMotor(0.0);
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return Math.abs(shooterSubsystem.getTargetRPM()-shooterSubsystem.getCurrentRPM()) <= (shooterSubsystem.getTargetRPM() * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {

  }
}