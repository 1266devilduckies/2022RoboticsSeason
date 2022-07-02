// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Fire2Balls;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopFlywheel;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Declare subsystems
  public static Drivetrain drivetrainSubsystem;
  public static Intake intakeSubsystem;
  public static Shooter shooterSubsystem;
  public static Climber climberSubsystem;

  //Define joysticks
  public final static Joystick driverJoystick = new Joystick(0);
  public final static Joystick operatorJoystick = new Joystick(1);

  //Define joystick buttons
  JoystickButton btn_ps4r1_driver = new JoystickButton(driverJoystick, 6);

  JoystickButton btn_ps4r1_operator = new JoystickButton(operatorJoystick, 6);


  private SendableChooser<Object[]> autonomousMode = new SendableChooser<Object[]>();

  private SequentialCommandGroup path1CommandGroup;
  private SequentialCommandGroup path2CommandGroup;
  private Trajectory auto1_path1;
  private Trajectory auto2_path1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystem definitions
    drivetrainSubsystem = new Drivetrain();
    intakeSubsystem = new Intake();
    shooterSubsystem = new Shooter();
    climberSubsystem = new Climber();

    //load in autonomous paths
    auto1_path1 = loadPath("auto1path1");
    auto2_path1 = loadPath("auto1path2");

    path1CommandGroup = new SequentialCommandGroup(generateTrajectoryCommand(auto1_path1), new SequentialCommandGroup(new Fire2Balls(shooterSubsystem), new StopFlywheel(shooterSubsystem)));
    path2CommandGroup = new SequentialCommandGroup(generateTrajectoryCommand(auto2_path1), new SequentialCommandGroup(new Fire2Balls(shooterSubsystem), new StopFlywheel(shooterSubsystem)));

    //Setup sendable chooser for autonomous mode selector
    Object[] data = {new SequentialCommandGroup(), null};
    autonomousMode.setDefaultOption("Do nothing", data);

    setAutonomousMode("1 Ball Auto", auto1_path1.getInitialPose(), path1CommandGroup);
    setAutonomousMode("2 Ball Auto", auto2_path1.getInitialPose(), path2CommandGroup);

    SmartDashboard.putData(autonomousMode);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //driver bindings
    btn_ps4r1_driver.whenPressed(new StartIntake(intakeSubsystem));
    btn_ps4r1_driver.whenReleased(new StopIntake(intakeSubsystem));

    //operator bindings
    btn_ps4r1_operator.whenPressed(new Fire2Balls(shooterSubsystem));
    btn_ps4r1_operator.whenReleased(new StopFlywheel(shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Object[] getAutonomousCommand() {
    return autonomousMode.getSelected();
  }

  private void setAutonomousMode(String inputName, Pose2d startingPose, SequentialCommandGroup commandsToDo) {
    Object[] data = {commandsToDo, startingPose};
    autonomousMode.addOption(inputName, data);
  }

  //Trajectory generator function
  private SequentialCommandGroup generateTrajectoryCommand(Trajectory trajectory) {
    if (trajectory != null) {
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
    } else {
      return new SequentialCommandGroup();
    }
}
private Trajectory loadPath(String address) {
  Trajectory trajectory = null;
  try {
    trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/"+address+".wpilib.json"));
  } catch(IOException exception) {
    DriverStation.reportError("Unable to open trajectory: " + address, exception.getStackTrace());
  }
  return trajectory;
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
  public static double RPMToEncoderTicksPer100ms(double rpm, double gearRatio, double CPR) {
    return ((rpm/60.0) * CPR * gearRatio) / 10.0;
  }

public static double EncoderTicksPer100msToRPM(double velocity, double gearRatio, double CPR) {
    return (((velocity*10.0)/CPR)/gearRatio) * 60.0;
}
}
