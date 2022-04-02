package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class StartFeeder extends CommandBase{
    
    public StartFeeder(){

    }

    double startTime;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        RobotMap.FeederMotor.set(ControlMode.Velocity, RobotMap.velocityFeeder);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return (System.currentTimeMillis() - startTime) >= 200;
    }

    @Override
    public void end(boolean interrupted){

        if(interrupted){
            RobotMap.inFiringCoroutine = false;
            RobotMap.isOneBall = false;
            RobotMap.pneumaticSingleSolenoid.set(false);
            RobotMap.overrideVelocity = -1.0;
            RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
            RobotMap.PewPewMotor2.set(ControlMode.Velocity, 0);
            RobotMap.PewPewMotor1.set(ControlMode.Velocity, 0);
        }
    }
}
