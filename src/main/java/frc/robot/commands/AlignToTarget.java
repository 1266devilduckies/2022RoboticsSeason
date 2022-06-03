package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class AlignToTarget extends CommandBase {
  Shooter shooterSubsystem;
  double limeLightDegreesAtStartup;
  double setpoint;
  double tickConversion;
  boolean aligning = false;
  public AlignToTarget(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    limeLightDegreesAtStartup = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    tickConversion = shooterSubsystem.getTurretPosition() + Constants.ticksPerDegreeTurret*limeLightDegreesAtStartup;
  }

  @Override
  public void execute() {
    limeLightDegreesAtStartup = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    tickConversion = shooterSubsystem.getTurretPosition() + Constants.ticksPerDegreeTurret*limeLightDegreesAtStartup;
    if (shooterSubsystem.isAtLowerBound(tickConversion) || shooterSubsystem.isAtUpperBound(tickConversion)) {
      shooterSubsystem.turretAlignmentMotor.set(ControlMode.PercentOutput, 0.0);
    } else if (!shooterSubsystem.isAtLowerBound(tickConversion) && !shooterSubsystem.isAtUpperBound(tickConversion)){
      shooterSubsystem.turretAlignmentMotor.set(ControlMode.Position, tickConversion);
    }
  }
  
  @Override
  public boolean isFinished() {
    double position = shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition();
    return Math.abs(position - tickConversion) < Constants.tickTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      shooterSubsystem.startedToBeAligned = true;
    }
  }
}