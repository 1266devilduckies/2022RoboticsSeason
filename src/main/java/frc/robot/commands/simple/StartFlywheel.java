package frc.robot.commands.simple;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ComputerVisionUtil;
import frc.robot.Constants;
import frc.robot.FlywheelInterpolator;
import frc.robot.LimeLight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  double rpm;
  public StartFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    double input = ComputerVisionUtil.calculateDistanceToTarget(Constants.limelightHeight, 
    Constants.hubHeight, Units.degreesToRadians(Constants.limelightMountAngle), 
    Units.degreesToRadians(LimeLight.getTy()), Units.degreesToRadians(LimeLight.getTx()));
    SmartDashboard.putNumber("distance to hub", input);

    double x = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getX();
    double y = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getY();
    double rot = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getRotation().getDegrees();
    //round to not clutter screen
    x = Math.floor(x*100)/100.;
    y = Math.floor(y*100)/100.;
    rot = Math.floor(rot*100)/100.;
    String robotPos = x + ", " + y + " - " + rot;
    SmartDashboard.putString("robot location relative to bottom left corner", robotPos);
    int idx = FlywheelInterpolator.findRangeIdx(input);
    rpm = FlywheelInterpolator.interpolateDataFromIdx(Constants.flywheelRPMData, 
    idx, 
    input); //4200
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