package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BetterKearnyDriving;
import frc.robot.commands.PewPewStart;
import frc.robot.commands.SlowShotChecker;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//This is basically our main class, we just don't use Main.java for clarity (i guess) -JM

public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static DriveSubsystem m_robotDrive;
  public static Intake intake;
  public static Shooter shooter;
  public static double turnY;
  public static double moveX;
  // in milliseconds
  // public long startAutoTime;
  // public long currentAutoTime = 0;
  // Field2d m_field = new Field2d();

  // trajectory
  // Trajectory trajectory = new Trajectory();
  // String trajectoryJSON = "paths/Auto1.wpilib.json";
  public static DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2), // 2 Falcon 500s on each side of the drivetrain.
      10, // Standard AndyMark Gearing reduction.
      2.1, // MOI of 2.1 kg m^2 (from CAD model).
      43.01, // Mass of the robot is 43.01 kg.
      Units.inchesToMeters(3), // Robot uses 3" radius (6" diameter) wheels.
      0.546,
      null);
  int i = 0;
  boolean finished = false;

  SequentialCommandGroup m_autonomousCommand;
  SendableChooser<Integer> autoRoutines;
  SendableChooser<Integer> autonomousTimeDelays;

  Trajectory auto1Part1;

  Trajectory auto2Part1;
  Trajectory auto2Part2;
  Trajectory auto2Part3;

  Trajectory auto3Part1;
  Trajectory auto3Part2;

  Trajectory auto4Part1;
  Trajectory auto4Part2;
  Trajectory auto4Part3;

  Trajectory auto5Part1;
  Trajectory auto5Part2;
  Trajectory auto5Part3;
  Trajectory auto5Part4;
  Trajectory auto5Part5;

  Trajectory auto6Part1;
  Trajectory auto6Part2;
  Trajectory auto6Part3;

  Trajectory auto7Part1;

  Trajectory auto8Part1;
  Trajectory auto8Part2;
  Trajectory auto8Part3;
  Trajectory auto8Part4;
  Trajectory auto8Part5;
  Trajectory auto8Part6;
  Trajectory auto8Part7;

  Trajectory auto1part1;
  Trajectory auto1part2;
  Trajectory auto1part3;
  Trajectory auto1part4;
  Trajectory auto1part5;
  Trajectory auto1part6;

  Trajectory auto0Part1;
  Trajectory auto0Part2;

  Trajectory initTrajectory;

  @Override
  public void robotInit() {
    RobotMap.init();
    autoRoutines = new SendableChooser<>();
    autonomousTimeDelays = new SendableChooser<>();
    m_robotDrive = new DriveSubsystem();
    drivetrain = new Drivetrain();

    try {
      initTrajectory = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto1Part1.wpilib.json"));
      auto1Part1 = initTrajectory;

      auto0Part1 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto0Part1.wpilib.json"));
        auto0Part2 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto0Part2.wpilib.json"));
      auto1part1 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part1.wpilib.json"));
        auto1part2 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part2.wpilib.json"));
        auto1part3 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part3.wpilib.json"));
        auto1part4 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part4.wpilib.json"));
        auto1part5 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part5.wpilib.json"));
        auto1part6 = TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/auto1part6.wpilib.json"));

      auto2Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto2Part1.wpilib.json"));
      auto2Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto2Part2.wpilib.json"));
      auto2Part3 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto2Part3.wpilib.json"));

      auto3Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto3Part1.wpilib.json"));
      auto3Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto3Part2.wpilib.json"));

      auto4Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto4Part1.wpilib.json"));
      auto4Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto4Part2.wpilib.json"));
      auto4Part3 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto4Part3.wpilib.json"));

      auto5Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto5Part1.wpilib.json"));
      auto5Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto5Part2.wpilib.json"));
      auto5Part3 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto5Part3.wpilib.json"));
      auto5Part4 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto5Part4.wpilib.json"));
      auto5Part5 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto5Part5.wpilib.json"));

      auto6Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto6Part1.wpilib.json"));
      auto6Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto6Part2.wpilib.json"));
      auto6Part3 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto6Part3.wpilib.json"));

      auto7Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto7Part1.wpilib.json"));

      auto8Part1 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part1.wpilib.json"));
      auto8Part2 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part2.wpilib.json"));
      auto8Part3 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part3.wpilib.json"));
      auto8Part4 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part4.wpilib.json"));
      auto8Part5 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part5.wpilib.json"));
      auto8Part6 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part6.wpilib.json"));
      auto8Part7 = TrajectoryUtil.fromPathweaverJson(
          Filesystem.getDeployDirectory().toPath().resolve("paths/auto8Part7.wpilib.json"));
    } catch (IOException ex) {
      // whoops
    }

    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorBack);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainLeftMotorFront);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorBack);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.MainRightMotorFront);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor1);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.PewPewMotor2);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.FeederMotor);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.Climber1);
    EncoderSetter.setEncoderDefaultPhoenixSettings(RobotMap.Climber2);
    // EncoderSim simEncoder = new EncoderSim(RobotMap.MainLeftMotorBack);
    RobotMap.PewPewMotor2.setInverted(true);
    // RobotMap.PewPewMotor2.setSensorPhase(true);
    RobotMap.PewPewMotor1.setInverted(false);
    RobotMap.PewPewMotor1.setSensorPhase(false);
    RobotMap.IntakeMotor1.setInverted(false);
    RobotMap.MainLeftMotorFront.setInverted(true);
    RobotMap.MainLeftMotorBack.setInverted(true);
    // they were false

    RobotMap.MainRightMotorFront.setInverted(false);
    //
    RobotMap.MainRightMotorBack.setInverted(false);

    RobotMap.Climber2.setInverted(true);
    RobotMap.Climber1.setNeutralMode(NeutralMode.Brake);
    RobotMap.Climber2.setNeutralMode(NeutralMode.Brake);
    RobotMap.MainLeftMotorBack.enableVoltageCompensation(false);
    RobotMap.MainLeftMotorFront.enableVoltageCompensation(false);
    RobotMap.MainRightMotorBack.enableVoltageCompensation(false);
    RobotMap.MainRightMotorFront.enableVoltageCompensation(false);

    RobotMap.MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
    RobotMap.MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
    RobotMap.MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
    RobotMap.MainRightMotorFront.setNeutralMode(NeutralMode.Coast);

    RobotMap.FeederMotor.setNeutralMode(NeutralMode.Brake);

    // SmartDashboard.putData("Field", m_field);

    RobotMap.gyro.calibrate();
    intake = new Intake();
    shooter = new Shooter();
    JoystickController.Init();
    // get auto path json
    autoRoutines.setDefaultOption("1 Ball Auto", 0);
    autoRoutines.addOption("Auto Defense", 1);
    autoRoutines.addOption("2 Ball Auto", 2);
    autoRoutines.addOption("4 Ball Auto", 3);
    SmartDashboard.putData(autoRoutines);

    autonomousTimeDelays.setDefaultOption("No delay", 0);
    autonomousTimeDelays.addOption("1 Second", 1000);
    autonomousTimeDelays.addOption("3 Seconds", 3000);
    autonomousTimeDelays.addOption("5 Seconds", 5000);
    SmartDashboard.putData(autonomousTimeDelays);

    if (!Preferences.containsKey("kP Shooter")) {
      Preferences.setDouble("kP Shooter", RobotMap.kP);
    }
    if (!Preferences.containsKey("kF Shooter")) {
      Preferences.setDouble("kF Shooter", RobotMap.kF);
    }
    if (!Preferences.containsKey("kP Feeder")) {
      Preferences.setDouble("kP Feeder", RobotMap.kP);
    }
    if (!Preferences.containsKey("kF Feeder")) {
      Preferences.setDouble("kF Feeder", RobotMap.kF);
    }
  }

  @Override
  public void robotPeriodic() {
    // gyro drift fix
    // if (++i > 3000 & !finished) {
    // SmartDashboard.putNumber("gyro error", RobotMap.gyro.getAngle());
    // finished = true;
    // }

    SmartDashboard.putNumber("climber one ticks", RobotMap.Climber1.getSelectedSensorPosition());

    DriveSubsystem.m_odometry.update(RobotMap.gyro.getRotation2d(),
        EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainLeftMotorBack.getSelectedSensorPosition()),
        EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.MainRightMotorBack.getSelectedSensorPosition()));

    double sdkP = Preferences.getDouble("kP Shooter", RobotMap.kP);
    double sdkf = Preferences.getDouble("kF Shooter", RobotMap.kF);
    if (RobotMap.kP != sdkP |
        RobotMap.kF != sdkf) {
      RobotMap.kP = sdkP;
      RobotMap.kF = sdkf;
      RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
      RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
      RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    } //all of these if statements are useless because the robot gets disabled when the preferences change lol
    double feederKpInp = Preferences.getDouble("kP Feeder", RobotMap.kPIndex);
    double feederKfInp = Preferences.getDouble("kF Feeder", RobotMap.kFIndex);
    if (RobotMap.kPIndex != feederKpInp |
        RobotMap.kFIndex != feederKfInp) {
      RobotMap.kPIndex = feederKpInp;
      RobotMap.kFIndex = feederKfInp;
      RobotMap.FeederMotor.config_kP(0, RobotMap.kPIndex);
      RobotMap.FeederMotor.config_kF(0, RobotMap.kFIndex);
    }
    double alignerKpInp = Preferences.getDouble("kP Aligner", RobotMap.kPAligner);
    double alignerKdInp = Preferences.getDouble("kD Aligner", RobotMap.kDAligner);
    if (RobotMap.kPAligner != alignerKpInp | RobotMap.kDAligner != alignerKdInp) {
      RobotMap.alignerController = new PIDController(RobotMap.kPAligner, 0.0, RobotMap.kDAligner);
    }

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  public SequentialCommandGroup generateTrajectoryCommand(Trajectory trajectory) {
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(RobotMap.kRamseteB, RobotMap.kRamseteZeta),
        new SimpleMotorFeedforward(
            RobotMap.ksVolts,
            RobotMap.kvVoltSecondsPerMeter,
            RobotMap.kaVoltSecondsSquaredPerMeter),
        RobotMap.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(RobotMap.kPDriveVel, 0, 0),
        new PIDController(RobotMap.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive);
    // m_robotDrive.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> m_robotDrive.arcadeDrive(0.0, 0.0));
  }

  public SequentialCommandGroup getAutonomousCommand(int num) {

    SequentialCommandGroup pathToGo = new SequentialCommandGroup();
    if(num == 0){
      Trajectory init = auto0Part1;
      m_robotDrive.resetOdometry(init.getInitialPose());
      pathToGo = new SequentialCommandGroup(generateTrajectoryCommand(auto0Part1), new PewPewStart(false), generateTrajectoryCommand(auto0Part2));
    }
    if (num == 1) {
      Trajectory init = auto1Part1;
      m_robotDrive.resetOdometry(init.getInitialPose());
      // move back and play defense
      pathToGo = new SequentialCommandGroup(generateTrajectoryCommand(auto1Part1));
    } else if (num == 2) {
      Trajectory init = auto2Part1;
      m_robotDrive.resetOdometry(init.getInitialPose());
      // 2 ball auto
      pathToGo = new SequentialCommandGroup(generateTrajectoryCommand(auto2Part1),
          new StartIntake(),
          generateTrajectoryCommand(auto2Part2),
          new StopIntake(),
          generateTrajectoryCommand(auto2Part3),
          new PewPewStart(false));
    } else if (num == 3) {
      // 4 ball auto
      Trajectory init = auto8Part1;
      m_robotDrive.resetOdometry(init.getInitialPose());
      pathToGo = new SequentialCommandGroup(new StartIntake(),
          generateTrajectoryCommand(init),
          new StopIntake(),
          generateTrajectoryCommand(auto8Part2),
          new PewPewStart(false),
          generateTrajectoryCommand(auto8Part3),
          new StartIntake(),
          generateTrajectoryCommand(auto8Part4),
          new StopIntake(),
          generateTrajectoryCommand(auto8Part5),
          new StartIntake(),
          generateTrajectoryCommand(auto8Part6),
          new StopIntake(),
          generateTrajectoryCommand(auto8Part7),
          new PewPewStart(false));
    }

    return pathToGo;
  }

  @Override
  public void autonomousInit() {
    RobotMap.PewPewMotor1.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor1.config_kP(0, RobotMap.kP);
    RobotMap.PewPewMotor2.config_kF(0, RobotMap.kF);
    RobotMap.PewPewMotor2.config_kP(0, RobotMap.kP);
    RobotMap.FeederMotor.config_kP(0, RobotMap.kPIndex);
    RobotMap.FeederMotor.config_kF(0, RobotMap.kFIndex);
    // startAutoTime = System.currentTimeMillis();
    m_autonomousCommand = getAutonomousCommand(autoRoutines.getSelected());
    // schedule the autonomous command (example)
    if (autonomousTimeDelays.getSelected() == 0) {
    CommandScheduler.getInstance().schedule(m_autonomousCommand);} else {
      CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
        new WaitUntilCommand(autonomousTimeDelays.getSelected()/1000.0),
      m_autonomousCommand));
    }
  }

  @Override
  public void teleopInit() {
  
    RobotMap.MainLeftMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainLeftMotorFront.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorBack.setSelectedSensorPosition(0);
    RobotMap.MainRightMotorFront.setSelectedSensorPosition(0);
    RobotMap.FeederMotor.setSelectedSensorPosition(0);
    RobotMap.PewPewMotor2.setSelectedSensorPosition(0);
    RobotMap.PewPewMotor1.setSelectedSensorPosition(0);

    RobotMap.m_drive.arcadeDrive(0.0, 0.0);
    RobotMap.Climber1.setSelectedSensorPosition(0);
    RobotMap.Climber2.setSelectedSensorPosition(0);
    // bind driver controls to the drivetrain
    // bind operator controls to climber
    CommandScheduler.getInstance().schedule(new BetterKearnyDriving(), new Climber(), new SlowShotChecker());
  }

  @Override
  public void teleopPeriodic() {
    // periodic events
    SmartDashboard.putNumber("climber one ticks", RobotMap.Climber1.getSelectedSensorPosition());
    limeLightDataFetcher.fetchData();

    // logging data
    /*
     * SmartDashboard.putBoolean("in coroutine", RobotMap.inFiringCoroutine);
     * SmartDashboard.putNumber("gyro rotation", RobotMap.gyro.getAngle());
     * SmartDashboard.putNumber("diffrence x",
     * limeLightDataFetcher.getdegRotationToTarget());
     * SmartDashboard.putNumber("difference y",
     * limeLightDataFetcher.getdegVerticalToTarget());
     */
  }

  @Override
  public void simulationPeriodic() {
    /*
     * Pass the robot battery voltage to the simulated Talon FXs /
     * Scheduler.getInstance().add(new BetterKearnyDriving());
     * // cod
     * /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     * 
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     * 
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    m_driveSim.setInputs(RobotMap.MainRightMotorBack.getMotorOutputPercent(),
        -RobotMap.MainLeftMotorBack.getMotorOutputPercent());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.update(0.001);

    /*
     * Update all of our sensors.
     * 
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    RobotMap.MainRightMotorBack.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainLeftMotorFront.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainRightMotorFront.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));
    RobotMap.MainLeftMotorBack.setSelectedSensorPosition(
        EncoderSetter.nativeUnitsToDistanceMeters(
            m_driveSim.getLeftPositionMeters()));

  }

  @Override
  public void autonomousPeriodic() {
    EncoderSetter.updateEncoders();
  }

  @Override
  public void testPeriodic() {
    RobotMap.Climber1.configForwardSoftLimitEnable(false, 0);
    RobotMap.Climber1.configReverseSoftLimitEnable(false, 0);
    RobotMap.Climber2.configForwardSoftLimitEnable(false, 0);
    RobotMap.Climber2.configReverseSoftLimitEnable(false, 0);
  }
}