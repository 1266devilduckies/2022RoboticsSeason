package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.subsystems.Shooter;

public class SeekTarget extends CommandBase {
  Shooter shooterSubsystem;
  boolean setpointIsLeft = false;
  public SeekTarget(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition() > 0) {
        setpointIsLeft = false;
    } else {
        setpointIsLeft = true;
    }
  }

  @Override
  public void execute() {
    double setpoint;
    if (setpointIsLeft == true) {
        setpoint = Constants.lowerBoundTicks;
    } else {
        setpoint = Constants.upperBoundTicks;
    }
    shooterSubsystem.turretAlignmentMotor.set(ControlMode.MotionMagic, setpoint);

    if (Math.abs(shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition() - setpoint) < Constants.tickTolerance) {
        setpointIsLeft = !setpointIsLeft;
    }
  }
  
  @Override
  public boolean isFinished() {
    boolean canSee = false;
    if (LimeLight.getTv() == 1.0) {
        canSee = true;
    }
    return canSee;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.turretAlignmentMotor.set(ControlMode.MotionMagic, 0);
    Shooter.startedSeeking = false;
  }
}