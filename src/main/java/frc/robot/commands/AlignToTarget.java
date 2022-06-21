package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class AlignToTarget extends CommandBase {
  Shooter shooterSubsystem;
  double setpoint;
  boolean aligning = false;
  public AlignToTarget(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (Robot.isSimulation()) {
    shooterSubsystem.turretAlignmentMotor.set(ControlMode.MotionMagic, Constants.ticksPerDegreeTurret*Drivetrain.limelightSim.getSimTx(0.0));
    } else {
      shooterSubsystem.turretAlignmentMotor.set(ControlMode.MotionMagic, Constants.ticksPerDegreeTurret*LimeLight.getTx());
    }
  }
  
  @Override
  public boolean isFinished() {
    boolean truthCondition = Math.abs(shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition() - Constants.ticksPerDegreeTurret*Drivetrain.limelightSim.getSimTx(0.0)) < Constants.tickTolerance;
    if (Robot.isReal()) {
      truthCondition = Math.abs(shooterSubsystem.turretAlignmentMotor.getSelectedSensorPosition() - Constants.ticksPerDegreeTurret*LimeLight.getTx()) < Constants.tickTolerance;
    }
    return truthCondition;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.startedToBeAligned = false;
  }
}