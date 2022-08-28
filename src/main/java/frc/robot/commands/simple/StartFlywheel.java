package frc.robot.commands.simple;

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
    System.out.println("started flywheel");
    shooterSubsystem.setRPM(Constants.flywheelRPM);
    shooterSubsystem.setIndexerMotor(0.0);
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return Math.abs(Constants.flywheelRPM - shooterSubsystem.getCurrentRPM()) <= (Constants.flywheelRPM * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {

  }
}