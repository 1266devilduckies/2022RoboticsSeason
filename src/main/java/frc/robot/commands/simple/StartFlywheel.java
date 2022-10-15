package frc.robot.commands.simple;

import org.opencv.core.Mat;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ComputerVisionUtil;
import frc.robot.Constants;
import frc.robot.FlywheelInterpolator;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.util.Units;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  double rpm;
  double overrideValue;
  boolean overrided = false;

  public StartFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    SmartDashboard.putNumber("target rpm", 0.0);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable datatable = inst.getTable("SmartDashboard");

    datatable.getEntry("target rpm").addListener(event -> {
      rpm = event.getEntry().getValue().getDouble();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    addRequirements(shooterSubsystem);
  }

  public StartFlywheel(Shooter subsystem, double overrideRpm) {
    overrided = true;
    overrideValue = overrideRpm;
  }

  @Override
  public void initialize() {
    double input = ComputerVisionUtil.calculateDistanceToTarget(Constants.limelightHeight,
        Constants.hubHeight, Units.degreesToRadians(Constants.limelightMountAngle),
        Units.degreesToRadians(LimeLight.getTy()), Units.degreesToRadians(-LimeLight.getTx()));

    int idx = FlywheelInterpolator.findRangeIdx(Constants.flywheelRPMData, input);
    if (!overrided) {
      rpm = FlywheelInterpolator.interpolateDataFromIdx(Constants.flywheelRPMData,
          idx,
          input);
    } else {
      rpm = overrideValue;
    }

    SmartDashboard.putNumber("target rpm real", rpm);
    shooterSubsystem.setRPM(rpm);
    shooterSubsystem.setIndexerMotor(0.0);
  }

  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    System.out.println("TARGET RPM: " + rpm);
    double normalizedSpeed = Math.abs(shooterSubsystem.getCurrentRPM());
    System.out.println("REAL RPM: " + normalizedSpeed);
    return Math.abs(rpm - normalizedSpeed) <= (rpm * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooterSubsystem.setToCoast();
    }
  }
}