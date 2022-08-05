package frc.robot.commands.simple;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OverrideAuto extends CommandBase{
   
    @Override
    public boolean isFinished() {
        return RobotContainer.driverJoystick.getRawButtonReleased(2);
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            SmartDashboard.putNumber("Override", 1);
            return;
        }

        SmartDashboard.putNumber("Override", 0);
    }
}
