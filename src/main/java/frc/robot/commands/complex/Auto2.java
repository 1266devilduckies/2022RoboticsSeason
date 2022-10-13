package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryFactories;
import frc.robot.commands.simple.AlignToTarget;
import frc.robot.commands.simple.StartIntake;
import frc.robot.commands.simple.StopIntake;

public class Auto2 extends SequentialCommandGroup {
    
    private static Trajectory firstPath = TrajectoryFactories.loadPath("auto2path1");

    public Auto2() {
        addCommands(
            new StartIntake(RobotContainer.intakeSubsystem),
            TrajectoryFactories.generateTrajectoryCommand(firstPath),
            new StopIntake(RobotContainer.intakeSubsystem),
            TrajectoryFactories.generateTrajectoryCommand(TrajectoryFactories.loadPath("auto2path2")),
            new AlignToTarget(RobotContainer.drivetrainSubsystem).withTimeout(3), 
            new FireBall(RobotContainer.shooterSubsystem),
            new FireBall(RobotContainer.shooterSubsystem)
            );
    }

    public static Pose2d getStartingPose() {
        return firstPath.getInitialPose();
    }
    
}
