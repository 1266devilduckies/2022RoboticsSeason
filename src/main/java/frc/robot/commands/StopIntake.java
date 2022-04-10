package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {
  Intake intakeSubsystem;
  long startTime;
  public StopIntake(Intake subsystem) {
    startTime = System.currentTimeMillis();
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
    return (System.currentTimeMillis() - startTime) >= Constants.actuatorFullyRetractedTimeMillis;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}