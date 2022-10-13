package frc.robot.commands.simple;

import org.opencv.core.Mat;

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
import edu.wpi.first.math.util.Units;

public class StartFlywheel extends CommandBase {
  Shooter shooterSubsystem;
  double rpm;
  public StartFlywheel(Shooter subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    double radians = Units.degreesToRadians(LimeLight.getTy() + Constants.limelightMountAngle);
    double input = (Constants.hubHeight - Constants.limelightHeight)/Math.tan(radians);//ComputerVisionUtil.calculateDistanceToTarget(Constants.limelightHeight, 
    //Constants.hubHeight, Units.degreesToRadians(Constants.limelightMountAngle), 
    //Units.degreesToRadians(LimeLight.getTy()), Units.degreesToRadians(-LimeLight.getTx()));
    SmartDashboard.putNumber("distance to hub", input);
    
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