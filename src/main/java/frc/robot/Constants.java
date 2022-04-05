// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //All constants are in SI units

    //Drivetrain CAN IDs
    public static final int CANID_mainLeftMotorBack = 0;
    public static final int CANID_mainLeftMotorFront = 1;

    public static final int CANID_mainRightMotorBack = 3;
    public static final int CANID_mainRightMotorFront = 2;

    //Gear ratio reductions
    public static final double GEARING_drivetrainGearbox = 8.333;

    //Voltage constants calculated from SysId on the drivetrain on the linear test
    public static final double kSLinear = 0.67766;
    public static final double kVLinear = 2.2804;
    public static final double kALinear = 0.6814;

    //Voltage constants calculated from SysId on the drivetrain on the angular test

    //test data not collected
    public static final double kSAngular = 0.067766;
    public static final double kVAngular = 0.22804;
    public static final double kAAngular = 0.06814;
    public static final double drivetrainWheelRadius = Units.inchesToMeters(2);
    public static final double trackWidth = Units.inchesToMeters(30);

    public static final double drivetrainSpeedLimiter = 0.8; //Limit the drivetrain to 80%
    public static final double driverJoystickDeadband = 0.05;
}
