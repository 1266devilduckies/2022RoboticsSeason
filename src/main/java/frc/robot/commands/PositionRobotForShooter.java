package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.limeLightDataFetcher;

public class PositionRobotForShooter extends CommandBase {
  HashMap<Double, Double[]> shooterDistances;
  double closestDistance = 0.05;
  PIDController movementController = new PIDController(0.02, 0.005, 0.01);
  //Proportional Gain | Feed-Forward Gain | Velocity
  Double[][] pfConstants = {
    {0.02, 0.045, 13250.0}, //0.5
    {0.03, 0.05, 20000.0} //3.0
  };
  Double[] targetConstants;
  double ty, tx, nearest, distance;
  final double limeLightMountDegreeHorizontal = 65.0;
  final double limeLightHeightFromCameraCircle = Units.inchesToMeters(28); //28 inches, edit this when they change the wheels
  final double topGoalHeight = Units.inchesToMeters(104); //8ft 8 in goal post height
  public PositionRobotForShooter() {
    shooterDistances = new HashMap<Double, Double[]>();
    shooterDistances.put(closestDistance, pfConstants[0]);
    shooterDistances.put(3.0, pfConstants[1]);
  }
  private double getLimelightDistance() {
    if (limeLightDataFetcher.seeIfTargetsExist() == 1.0) {
    ty = limeLightDataFetcher.getdegVerticalToTarget();
    tx = limeLightDataFetcher.getdegRotationToTarget();
    double angleToGoal = limeLightMountDegreeHorizontal + ty;
    double angleToGoalRadians = Math.toRadians(angleToGoal);
    return Math.abs((topGoalHeight - limeLightHeightFromCameraCircle)/Math.tan(angleToGoalRadians));
    } else {
      return -1.0;
    }
  }
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    nearest = closestDistance;
    RobotMap.limeLightDistance = getLimelightDistance();
    for (HashMap.Entry<Double, Double[]> entry : shooterDistances.entrySet()) {
      double key = entry.getKey();
      Double[] value = entry.getValue();

      if (Math.abs(key-RobotMap.limeLightDistance) < nearest) {
        nearest = key;
        targetConstants = value;
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    RobotMap.limeLightDistance = getLimelightDistance();
    double pidOutput = movementController.calculate(RobotMap.limeLightDistance, nearest);
    RobotMap.m_drive.tankDrive(pidOutput, pidOutput, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotMap.limeLightDistance-nearest) < 0.1) || RobotMap.limeLightDistance < 0 /*limelight cant see top*/;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    RobotMap.overrideVelocity = targetConstants[2];
    RobotMap.m_drive.tankDrive(0.0, 0.0);
  }

}// class
