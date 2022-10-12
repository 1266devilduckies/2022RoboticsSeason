package frc.robot.commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.simple.IndexBall;
import frc.robot.commands.simple.StartFlywheel;
import frc.robot.commands.simple.StopFlywheel;
import frc.robot.subsystems.Shooter;

public class FireBall extends SequentialCommandGroup {
    public FireBall(Shooter shooterSubsystem) {
        addCommands(
        new StartFlywheel(shooterSubsystem), 
        new IndexBall(shooterSubsystem),
        new StopFlywheel(shooterSubsystem));
    }
}
