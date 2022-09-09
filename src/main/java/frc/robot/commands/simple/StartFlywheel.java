package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  double rpm = 4200;
  public StartFlywheel(Shooter subsystem) {
    SmartDashboard.putNumber("flywheel rpm", 4200);
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    rpm = SmartDashboard.getNumber("flywheel rpm", 4200);
    shooterSubsystem.setRPM(rpm);
    System.out.println(rpm);
    shooterSubsystem.setIndexerMotor(0.0);
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    System.out.println(Math.abs(rpm - shooterSubsystem.getCurrentRPM()));
    return Math.abs(rpm - shooterSubsystem.getCurrentRPM()) <= (rpm * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {

  }
}