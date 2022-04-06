// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrainSubsystem;
  private final Command autonomousCommand;

  public final static Joystick driverJoystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //It's best to define your subsystems and commands here as any constructor code will get ran on robotInit, which could prevent unexpected behavior
    drivetrainSubsystem = new Drivetrain();
    autonomousCommand = new SequentialCommandGroup(); //default to an autonomous which does nothing
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousCommand;
  }

  //Trajectory generator function
  public SequentialCommandGroup generateTrajectoryCommand(Trajectory trajectory) {
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.kSLinear,
                Constants.kVLinear,
                Constants.kALinear),
            Constants.kDriveKinematics,
            drivetrainSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrainSubsystem::tankDriveVolts,
            drivetrainSubsystem);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
}
  //Conversion functions

  //Gear ratio must be a reduction, CPR means ticks per revolution
  public static double encoderTicksToMeters(double ticks, double gearRatio, double CPR, double wheelRadius) {
    double numAxleRotations = ticks/CPR; //is a form of angular velocity, so Win/Wout works
    double numRotationsOnDriverGear = numAxleRotations/gearRatio; //Din/Dout = gear ratio
    //2pi is the circumfrence on unit circle
    //multiply it to scale it and multiply that UNIT of rotation to distance by the total amount of rotations
    return numRotationsOnDriverGear*2*Math.PI*wheelRadius;
  }
  public static double metersToEncoderTicks(double meters, double gearRatio, double CPR, double wheelRadius) {
   //rearrange the equation in encoderTicksToMeters and solve for ticks
   return (meters/(2*Math.PI*wheelRadius))*gearRatio*CPR;
  }
  public static double metersPerSecondToEncoderTicksPer100ms(double metersPerSecond, double gearRatio, double CPR, double wheelRadius) {
    //convert to meters per 100 ms
    metersPerSecond /= 10.0;
    return metersToEncoderTicks(metersPerSecond, gearRatio, CPR, wheelRadius);
  }
  public static double EncoderTicksPer100msToMetersPerSecond(double ticksPer100ms, double gearRatio, double CPR, double wheelRadius) {
    //convert to encoder ticks per second
    return encoderTicksToMeters(ticksPer100ms*10.0, gearRatio, CPR, wheelRadius);
  }
}
