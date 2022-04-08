package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {
  Intake intakeSubsystem;
  public StopIntake(Intake subsystem) {
    intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setSolenoid(false);
    intakeSubsystem.setIntakeMotor(0.0);
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