package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.RobotMap;

public class SlowShot extends Command {

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        RobotMap.fullShooterPower = false;
        Scheduler.getInstance().add(new PewPewStart());
    } 

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        
    }

}