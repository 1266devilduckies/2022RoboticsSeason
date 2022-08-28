package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class OverrideBtnPressed extends CommandBase {
  public OverrideBtnPressed() {
  }

  @Override
  public void initialize() {
      System.out.println("updated");
    Shooter.timeSinceOverridedAutonomous = Timer.getFPGATimestamp();
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