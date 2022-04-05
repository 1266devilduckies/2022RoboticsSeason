package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.JoystickController;
import frc.robot.RobotMap;

public class SlowShotChecker extends CommandBase {

    boolean alreadyRan = false;

    // Called just before this Command runs the first time

    @Override
    public void initialize() {
        
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        boolean isPressed = JoystickController.COPILOT_JOYSTICK.RightTriggeredPressed();

        if(isPressed && !RobotMap.inFiringCoroutine){
           // CommandScheduler.getInstance().schedule(new PewPewStart(true));
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run

}