package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class PewPewEnd extends CommandBase{
    
    public PewPewEnd() {

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override
    public void end(boolean interrupted){
        RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
        RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
        RobotMap.PewPewMotor1.set(ControlMode.Velocity, 0);
    }
}
