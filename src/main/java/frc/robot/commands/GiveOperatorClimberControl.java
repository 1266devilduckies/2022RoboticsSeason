package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class GiveOperatorClimberControl extends CommandBase{

    public GiveOperatorClimberControl() {
        // Use requires() here to declare subsystem dependencies
    
    }
    
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        RobotMap.operatorIsControlling = true;
    }
}