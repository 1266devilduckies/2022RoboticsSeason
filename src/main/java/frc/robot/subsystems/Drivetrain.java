package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
//import jdk.jfr.Percentage;

public class Drivetrain extends Subsystem {
	TalonFX MainLeftMotor = RobotMap.MainLeftMotorBack;
	TalonFX MainRightMotor = RobotMap.MainRightMotorBack;
	TalonFX SlaveLeftMotor = RobotMap.MainLeftMotorFront;
	TalonFX SlaveRightMotor = RobotMap.MainRightMotorFront;


	/*public void tankDriveVoltage(double left, double right, double maxX, double maxY) {
		left *= maxX;//left joystick x axis    -  to invert controls, make one of these negative
		right *= maxY;//right joystick y axis
		
		MainLeftMotor.set(ControlMode.PercentOutput,left);
		MainRightMotor.set(ControlMode.PercentOutput,right);
		/*SlaveLeftMotor.set(ControlMode.PercentOutput,left);
		SlaveRightMotor.set(ControlMode.PercentOutput,right);*/
	//}
	public void arcadeDriveVoltage(double x, double y, double maxX, double maxY) {
		x *= maxX;
		y *= maxY;
		x *= -1;
		double left = y + x;
		double right = y - x;
		if(Math.abs(left) > 1){
			left /= Math.abs(left);
			right /= Math.abs(left);
		}
		else if(Math.abs(right) > 1){
			left /= Math.abs(right);
            right /= Math.abs(right);
		}
		MainLeftMotor.set(ControlMode.PercentOutput,left);
		MainRightMotor.set(ControlMode.PercentOutput,right);
		SlaveLeftMotor.set(ControlMode.PercentOutput,left);
		SlaveRightMotor.set(ControlMode.PercentOutput,right);
	}
         public double rotateToAngle(double targetAngle, double gyroAngle, double threshold) {
			double error = targetAngle - gyroAngle; 
			double rotation = 0;
			if (Math.abs(error) > threshold){
				rotation = error * 0.225;
				return rotation;
				}
				else {
				rotation = -0.5;
				return rotation;	
				}
		 }
		 @Override
		 public void initDefaultCommand(){
        }   
	}
