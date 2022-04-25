package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  public StartFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println(Constants.PID_kF_flywheel);
    shooterSubsystem.setTargetRPM(Constants.flywheelRPM);
    shooterSubsystem.setRPM(shooterSubsystem.getTargetRPM());
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return false;//Math.abs(shooterSubsystem.getTargetRPM()-shooterSubsystem.getCurrentRPM()) <= (shooterSubsystem.getTargetRPM() * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {

  }
}