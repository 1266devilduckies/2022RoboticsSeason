// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
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

  //Declare commands
  public static Command StartIntake;
  public static Command StopIntake;

  //Define joysticks
  public final static Joystick driverJoystick = new Joystick(0);
  public final static Joystick operatorJoystick = new Joystick(1);

  //Define joystick buttons
  JoystickButton btn_ps4r1_driver = new JoystickButton(driverJoystick, 6);

  JoystickButton btn_ps4r1_operator = new JoystickButton(operatorJoystick, 6);


  private SendableChooser<SequentialCommandGroup> autonomousMode = new SendableChooser<SequentialCommandGroup>();
  
  private Trajectory firstTrajectoryInAutonomous; //needs to have its odometry updated for it to work not only in robot simulation, but for it to respect symmetry of the field irl

  private SequentialCommandGroup path1CommandGroup;
  private Trajectory path1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystem definitions
    drivetrainSubsystem = new Drivetrain();
    intakeSubsystem = new Intake();
    shooterSubsystem = new Shooter();

    //Intake definitions
    StartIntake = new StartIntake(intakeSubsystem);
    StopIntake = new StopIntake(intakeSubsystem);

    //load in autonomous paths
    path1 = loadPath("path1");
    path1CommandGroup = generateTrajectoryCommand(path1);

    //Setup sendable chooser for autonomous mode selector
    autonomousMode.setDefaultOption("Do nothing", new SequentialCommandGroup());

    setAutonomousMode("1 Ball Auto", path1, new SequentialCommandGroup(path1CommandGroup));

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
    btn_ps4r1_driver.whenPressed(StartIntake);
    btn_ps4r1_driver.whenReleased(StopIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    drivetrainSubsystem.resetOdometry(firstTrajectoryInAutonomous.getInitialPose());
    return autonomousMode.getSelected();
  }

  private void setAutonomousMode(String inputName, Trajectory firstPathInAutoMode, SequentialCommandGroup commandsToDo) {
    firstTrajectoryInAutonomous = firstPathInAutoMode;
    autonomousMode.addOption(inputName, commandsToDo);
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
    return (rpm/60.0) * CPR * gearRatio * 10.0;
  }

public static double EncoderTicksPer100msToRPM(double velocity, double gearingDrivetraingearbox, double CPR) {
    return (((velocity*10.0)/CPR)/Constants.GEARING_drivetrainGearbox) * 60.0;
}
}
