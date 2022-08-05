package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class Path1Auto extends SequentialCommandGroup {
    
    public Path1Auto(Shooter shooterSubsystem) {
        addCommands(
            new ParallelRaceGroup(generateTrajectoryCommand(auto1_path1), new OverrideAuto()), 
            new ParallelRaceGroup(new SequentialCommandGroup(new Fire2Balls(shooterSubsystem), new StopFlywheel(shooterSubsystem)), new OverrideAuto()))
        );
    }
    
}
