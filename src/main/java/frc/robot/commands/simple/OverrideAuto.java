package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class OverrideAuto extends CommandBase{
   
    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - Shooter.timeSinceOverridedAutonomous) > 1) && (Shooter.timeSinceOverridedAutonomous > 0);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.timeSinceOverridedAutonomous = -1;
        if(!interrupted){
            SmartDashboard.putNumber("Override", 1);
            return;
        }

        SmartDashboard.putNumber("Override", 0);
    }
}
