/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static Spark LeftDriveMotor1;
  public static Spark LeftDriveMotor2;
  public static Spark RightDriveMotor1;
  public static Spark RightDriveMotor2;
  public static Talon UpDriveMotor1; 
//public static Talon IntakeDriveMotor1;


  public static void init() {
    LeftDriveMotor1 = new Spark(2);
    LeftDriveMotor2 = new Spark(3);
    RightDriveMotor1 = new Spark(1);
    RightDriveMotor2 = new Spark(0);
    UpDriveMotor1 = new Talon(4);
//IntakeDriveMotor1 = new Talon(5);

  }
}
