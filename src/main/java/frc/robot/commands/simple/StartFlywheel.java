package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    double input = (double)LimeLight.getRobotPoseFromVision()[1];
    SmartDashboard.putNumber("distance to hub", input);

    double x = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getX();
    double y = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getY();
    double rot = RobotContainer.drivetrainSubsystem.odometry.getEstimatedPosition().getRotation().getDegrees();
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
    System.out.println(Math.abs(rpm - shooterSubsystem.getCurrentRPM()));
    return Math.abs(rpm - shooterSubsystem.getCurrentRPM()) <= (rpm * Constants.flywheelTolerance);
  }

  @Override
  public void end(boolean interrupted) {

  }
}