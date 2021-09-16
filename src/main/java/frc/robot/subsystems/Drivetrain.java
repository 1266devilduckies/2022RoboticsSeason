/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import  edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  Spark leftDrive1 = RobotMap.LeftDriveMotor1;
  Spark RightDrive1 = RobotMap.RightDriveMotor1;
  Spark leftDrive2 = RobotMap.LeftDriveMotor2;
  Spark RightDrive2 = RobotMap.RightDriveMotor2;

    // TalonFX leftDriveTrainer = RobotMap.leftDriveTrainer;
  //// TalonFX rightDriveTrainer = RobotMap.rightDriveTrainer;
  // TalonFX leftDrivePokemon = RobotMap.leftDrivePokemon;
  // TalonFX rightDrivePokemon = RobotMap.rightDrivePokemon;

	//TestDrive
	//public static TalonFX TestLeftM = RobotMap.TestLeftMaster;
  //public static TalonFX TestRightM = RobotMap.TestRightMaster;

 
  
	public void arcadeDriveVoltage(double x, double y, double maxX, double maxY) {
		x *= maxX;
		y *= maxY;
		x *= -1;
		double left = y + x;
		double right = y - x;
		if(Math.abs(left) > 1) {
			left /= Math.abs(left);
			right /= Math.abs(left);
		}
		else if(Math.abs(right) > 1) {
			left /= Math.abs(right);
			right /= Math.abs(right);
		}
		leftDrive1.set(left);
		RightDrive1.set(-right);
		leftDrive2.set(left);
		RightDrive2.set(right);
		//leftDriveTrainer.set(ControlMode.PercentOutput, left);
		//rightDriveTrainer.set(ControlMode.PercentOutput, right);
		//leftDrivePokemon.set(ControlMode.PercentOutput, left);
		//rightDrivePokemon.set(ControlMode.PercentOutput, right);
		//TestLeftM.set(ControlMode.PercentOutput, left);
		//TestRightM.set(ControlMode.PercentOutput, right);
	}
	public double rotateToAngle(double targetAngle, double gyroAngle, double threshold) {
		double error = targetAngle - gyroAngle; 
		double rotation  = 0;
		if (Math.abs(error) > threshold){
			rotation =  error* 0.45; //remember to set this to the kp value 
			return rotation;
		} else {
			rotation = -1;
			return rotation;
		}
			
	}
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
	}

	public void dontCrash(boolean aboutToCrash) {
		if (aboutToCrash) {
			dont();
		}
	}

	public void dont() {
		//just don't
	}
}
