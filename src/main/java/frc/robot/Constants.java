// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

    //Pneumatics CAN IDs
    public static final int CANID_pneumaticsControlModule = 10;
    public static final int intakeSolenoidPort = 0;
    public static final int CANID_intakeMotor = 4;

    //Drivetrain CAN IDs
    public static final int CANID_mainLeftMotorBack = 0;
    public static final int CANID_mainLeftMotorFront = 1;

    public static final int CANID_mainRightMotorBack = 3;
    public static final int CANID_mainRightMotorFront = 2;

    //Shooter CAN IDs
    public static final int CANID_leftFlywheelMotor = 8;
    public static final int CANID_rightFlywheelMotor = 5;

    public static final int CANID_indexerMotor = 9;

    public static final int CANID_turretAlignmentMotor = 14;

    //Climber CAN IDs

    //check in on these when you get the chance
    public static final int CANID_leftClimberMotor = 13;
    public static final int CANID_rightClimberMotor = 12;
    public static double upperBoundClimber = -150000.0; //negative is up
    public static double lowerBoundClimber = 0.0; //ten thousand

    //Turret
    public static final double PID_kP_turretAlignment = 0.05;
    public static final double PID_kD_turretAlignment = 0.0;
    public static final double kSTurret = 0.58044;
    public static final double kVTurret = 7.7555;
    public static final double kATurret = 0.19171;
    public static final double ticksPerDegreeTurret = (2048*Constants.GEARING_turret)/360.;
    public static final double lowerBoundTicks = Constants.lowerBoundShooterDegrees * ticksPerDegreeTurret;
    public static final double upperBoundTicks = Constants.upperBoundShooterDegrees * ticksPerDegreeTurret;
    public static final double tickTolerance = ticksPerDegreeTurret * 0.5;
    public static final Translation2d hubPosition = new Translation2d(Units.feetToMeters(54/2.), Units.feetToMeters(27/2.));
    public static final double limelightHorizontalRange = 27; //in terms of degrees
        //43 in from cart
    public static final double hubHeight = 2.6416;
    public static final double limelightHeight = Units.inchesToMeters(30);
    public static final double limelightMountAngle = 24; // real pls no change :pray:
    //in terms of units to meters with a field centric position with a robot pose of 0,0, (0,1)
    public static final Transform2d limelightOffsetFromCenterRobot = new Transform2d(
      new Translation2d(             
      Units.inchesToMeters(12),
      0),
      Rotation2d.fromDegrees(0)
    );

    //Gear ratio reductions
    public static final double GEARING_drivetrainGearbox = 8.333;
    public static final double GEARING_turret = 74.0;
                                                                                            
    //Flywheel
    public static final double flywheelRPM = 4200;
    public static final double kSFlywheel = 0.50745;
    public static final double kVFlywheel = 0.10828;
    public static final double kAFlywheel = 0.0080391;
    public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEEDFORWARD_flywheel = new SimpleMotorFeedforward(kSFlywheel, kVFlywheel, kAFlywheel); //in terms of rps
    public static final double PID_kP_flywheel = 0.1;
    public static final double flywheelTolerance = 0.02; //2 percent plus or minus

    //assumed with 0 degrees being heading
    public static final double lowerBoundShooterDegrees = -190;
    public static final double upperBoundShooterDegrees = 190;

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
    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double robotWeight = Units.lbsToKilograms(140);

    public static final double drivetrainSpeedLimiter = .8; //Limit the drivetrain to 80%
    public static final double driverJoystickDeadband = 0.05;

    public static final double kMaxSpeedMetersPerSecond = 1.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.75;
    public static final double kPDriveVel = 3.473;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      trackWidth);

    //timing constants
    public static final long actuatorFullyExtendedTimeMillis = 1000;
    public static final long actuatorFullyRetractedTimeMillis = 1000;
    public static final int indexingTimeMillis = 500;
    public static final long LOSScanMillis = 1000;
    public static final double kS_turret = 0;
    public static final double kV_turret = 0;
    public static final double kA_turret = 0;

    //interpolation data
    public static final double[][] flywheelRPMData = {
      //format of rpm, distance for that rpm
      {3950, 2.64},
      {4000, 2.77},
      {4300, 3.1},
      {4600, 3.22},
      {4700, 3.33},
      {4850,3.51},
      {4950, 3.69}
    };

    //when there two ball
    public static final double[][] flywheelRPMDataCompressed = {
      //format of rpm, distance for that rpm
      {},
      {},
      {},
    };
}
