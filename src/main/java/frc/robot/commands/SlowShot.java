package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;

public class SlowShot extends Command {

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        if (!RobotMap.inFiringCoroutine) {
            RobotMap.inFiringCoroutine = true;
            RobotMap.fullShooterPower = false;
            RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF2);
            RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP2);
            RobotMap.PewPewMotor1.config_kI(0, RobotMap.kI2);
            RobotMap.PewPewMotor1.config_kD(0, RobotMap.kD2);
            RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF2);
            RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP2);
            RobotMap.PewPewMotor2.config_kI(0, RobotMap.kI2);
            RobotMap.PewPewMotor2.config_kD(0, RobotMap.kD2);
            RobotMap.timeSinceStartedBeingReleasedForShooter = System.currentTimeMillis();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        /*
         * RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF2);
         * RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP2);
         * RobotMap.PewPewMotor1.config_kI(0, RobotMap.kI2);
         * RobotMap.PewPewMotor1.config_kD(0, RobotMap.kD2);
         * RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF2);
         * RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP2);
         * RobotMap.PewPewMotor2.config_kI(0, RobotMap.kI2);
         * RobotMap.PewPewMotor2.config_kD(0, RobotMap.kD2);
         */

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
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