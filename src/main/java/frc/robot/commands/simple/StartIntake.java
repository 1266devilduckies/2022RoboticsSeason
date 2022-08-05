package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class StartIntake extends CommandBase {
  Intake intakeSubsystem;
  double startTime;
  public StartIntake(Intake subsystem) {
    intakeSubsystem = subsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setSolenoid(true);
    intakeSubsystem.setIntakeMotor(1.0);
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {

  }
  
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime) >= Constants.actuatorFullyExtendedTimeMillis /1000.;
  }

  @Override
  public void end(boolean interrupted) {
    
  }
}