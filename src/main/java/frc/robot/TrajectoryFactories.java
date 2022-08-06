package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TrajectoryFactories {
  public static SequentialCommandGroup generateTrajectoryCommand(Trajectory trajectory) {
    if (trajectory != null) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            RobotContainer.drivetrainSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.kSLinear,
                Constants.kVLinear,
                Constants.kALinear),
            Constants.kDriveKinematics,
            RobotContainer.drivetrainSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            RobotContainer.drivetrainSubsystem::tankDriveVolts,
            RobotContainer.drivetrainSubsystem);
    
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> RobotContainer.drivetrainSubsystem.tankDriveVolts(0, 0));
    } else {
      return new SequentialCommandGroup();
    }
}

public static Trajectory loadPath(String address) {
  Trajectory trajectory = null;
  try {
    trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/"+address+".wpilib.json"));
  } catch(IOException exception) {
    DriverStation.reportError("Unable to open trajectory: " + address, exception.getStackTrace());
  }
  return trajectory;
}
}
