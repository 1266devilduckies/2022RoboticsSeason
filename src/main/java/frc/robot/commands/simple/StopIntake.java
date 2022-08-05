package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {
  Intake intakeSubsystem;
  double startTime;
  public StopIntake(Intake subsystem) {
    intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setSolenoid(false);
    intakeSubsystem.setIntakeMotor(0.0);
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) >= Constants.actuatorFullyRetractedTimeMillis/1000.;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}