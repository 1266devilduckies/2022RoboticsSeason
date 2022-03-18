package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotMap;

public class SlowShot extends CommandBase {

    // Called just before this Command runs the first time
    CommandBase pewpewStartCommand;

    @Override
    public void initialize() {
        RobotMap.fullShooterPower = false;
        pewpewStartCommand = new PewPewStart(true);
        CommandScheduler.getInstance().schedule(pewpewStartCommand);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        //CommandScheduler.getInstance().cancel(pewpewStartCommand);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

}