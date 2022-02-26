package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.RobotMap;
import frc.robot.EncoderSetter;
//import jdk.jfr.Percentage;

public class Drivetrain extends Subsystem {

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
		RobotMap.MainLeftMotorBack.set(ControlMode.PercentOutput, left);
		RobotMap.MainRightMotorBack.set(ControlMode.PercentOutput, right);
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

	@Override
	public void initDefaultCommand() {

	}
}
