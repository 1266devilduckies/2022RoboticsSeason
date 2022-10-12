// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.complex.Auto1;
import frc.robot.commands.complex.Auto2;
import frc.robot.commands.complex.FireBall;
import frc.robot.commands.simple.AlignToTarget;
import frc.robot.commands.simple.IndexBall;
import frc.robot.commands.simple.OverrideAuto;
import frc.robot.commands.simple.StartIntake;
import frc.robot.commands.simple.StopFlywheel;
import frc.robot.commands.simple.StopIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  JoystickButton btn_ps4x_driver = new JoystickButton(driverJoystick, 2);
  JoystickButton btn_ps4L2_driver = new JoystickButton(driverJoystick, 7);

  JoystickButton btn_ps4r1_operator = new JoystickButton(operatorJoystick, 6);


  private SendableChooser<Object[]> autonomousMode = new SendableChooser<Object[]>();

  private SequentialCommandGroup path1CommandGroup;
  private SequentialCommandGroup path2CommandGroup;

  public static boolean overriding = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Subsystem definitions
    drivetrainSubsystem = new Drivetrain();
    intakeSubsystem = new Intake();
    shooterSubsystem = new Shooter();
    climberSubsystem = new Climber();

    path1CommandGroup = new Auto1();
    path2CommandGroup = new Auto2();

  Object[] data = {new SequentialCommandGroup(), new Pose2d(0,0, Rotation2d.fromDegrees(0))}; //new Pose2d(0,0, Rotation2d.fromDegrees(0))}; //spec [command to run at auto, starting position for odometry]
    autonomousMode.setDefaultOption("Do nothing", data);

    setAutonomousMode("1 Ball Auto", Auto1.getStartingPose(), path1CommandGroup);
    setAutonomousMode("2 Ball Auto", Auto2.getStartingPose(), path2CommandGroup);

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

    //btn_ps4x_driver.whenPressed(new OverrideBtnPressed());
    btn_ps4x_driver.whenPressed(new IndexBall(shooterSubsystem));
    btn_ps4L2_driver.whileHeld(new AlignToTarget(drivetrainSubsystem), true);
    //driver bindings
    btn_ps4r1_driver.whenPressed(new StartIntake(intakeSubsystem));
    btn_ps4r1_driver.whenReleased(new StopIntake(intakeSubsystem));

    //operator bindings
    btn_ps4r1_operator.whenPressed(new FireBall(shooterSubsystem));
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

  public static ParallelRaceGroup bindOverride(Command command) {
    return new ParallelRaceGroup(command, new OverrideAuto());
  }

}
