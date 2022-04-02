package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class StopFeeder extends CommandBase{
    
    public StopFeeder(){

    }
    double startTime;

    @Override
    public void initialize(){
        startTime = System.currentTimeMillis();
        RobotMap.FeederMotor.set(ControlMode.Velocity, 0);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return (((RobotMap.PewPewMotor2.getSelectedSensorVelocity() ) - RobotMap.velocityCurrent) >= 0) &&
            (((RobotMap.PewPewMotor1.getSelectedSensorVelocity() ) - RobotMap.velocityCurrent) >= 0);
    }

    @Override
    public void end(boolean interrupted) {
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
