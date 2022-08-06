package frc.robot.commands.complex;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryFactories;

public class Auto2 extends SequentialCommandGroup {
    
    private static Trajectory firstPath = TrajectoryFactories.loadPath("auto1path2");

    public Auto2() {
        addCommands(
            RobotContainer.bindOverride(TrajectoryFactories.generateTrajectoryCommand(firstPath)), 
            RobotContainer.bindOverride(new Fire2Balls(RobotContainer.shooterSubsystem))
            );
    }

    public static Pose2d getStartingPose() {
        return firstPath.getInitialPose();
    }
    
}
