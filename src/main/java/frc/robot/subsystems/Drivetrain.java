package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;

//import jdk.jfr.Percentage;

public class Drivetrain extends SubsystemBase {

	public void arcadeDriveVoltage(double x, double y, double maxX, double maxY) {
		x *= maxX;
		y *= maxY;
		x *= -1;
		double left = y + x;
		double right = y - x;
		if (Math.abs(left) > 1) {
			left /= Math.abs(left);
			right /= Math.abs(left);
		} else if (Math.abs(right) > 1) {
			left /= Math.abs(right);
			right /= Math.abs(right);
		}
		// fix deadband
		// if (x > 0.05 || y > 0.05) {
		RobotMap.MainLeftMotorBack.set(ControlMode.PercentOutput, left);
		RobotMap.MainRightMotorBack.set(ControlMode.PercentOutput, right);
		// } else {
		// RobotMap.MainLeftMotorBack.set(ControlMode.PercentOutput, 0.0);
		// RobotMap.MainRightMotorBack.set(ControlMode.PercentOutput, 0.0);
		// }
	}

	public double rotateToAngle(double targetAngle, double gyroAngle, double threshold) {
		double error = targetAngle - gyroAngle;
		double rotation = 0;
		if (Math.abs(error) > threshold) {
			rotation = error * 0.225;
			return rotation;
		} else {
			rotation = -0.5;
			return rotation;
		}
	}

	public void initDefaultCommand() {

	}
}
