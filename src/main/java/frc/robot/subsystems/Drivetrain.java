package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ComputerVisionUtil;
import frc.robot.Constants;
import frc.robot.GearUtil;
import frc.robot.LimeLight;
import frc.robot.LineRenderer;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  public final DifferentialDrive robotDrive;
  private final DifferentialDrivetrainSim robotDriveSim;

  //CAN devices
  private final WPI_TalonFX MainLeftMotorBack;
  private final WPI_TalonFX MainRightMotorBack;
  private final WPI_TalonFX MainLeftMotorFront;
  private final WPI_TalonFX MainRightMotorFront;

  //CAN Sim collections, have the ability to get and set the encoder position and velocity without actually changing the actual CAN devices. 
  //You only really need two for are case, that being the masters
  private final TalonFXSimCollection leftMotorSim;
  private final TalonFXSimCollection rightMotorSim;

  private final ADXRS450_GyroSim gyroSim;
  public ADXRS450_Gyro gyro;

  public DifferentialDrivePoseEstimator odometry;
  
  public final static Field2d field = new Field2d(); //used to simulate the field for the simulated robot
  public LimeLight limelightSim;
  private LineRenderer turretDirection;

  public Drivetrain() {
    MainLeftMotorBack = new WPI_TalonFX(Constants.CANID_mainLeftMotorBack);
    MainRightMotorBack = new WPI_TalonFX(Constants.CANID_mainRightMotorBack);
    MainLeftMotorFront = new WPI_TalonFX(Constants.CANID_mainLeftMotorFront);
    MainRightMotorFront = new WPI_TalonFX(Constants.CANID_mainRightMotorFront);

    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    gyroSim = new ADXRS450_GyroSim(gyro);
    odometry = new DifferentialDrivePoseEstimator(Rotation2d.fromDegrees(-gyro.getAngle()),
     new Pose2d(5.0, 8.0, new Rotation2d()),  
     new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.);
    
    //Reset settings
    MainLeftMotorBack.configFactoryDefault();
    MainRightMotorBack.configFactoryDefault();
    MainLeftMotorFront.configFactoryDefault();
    MainRightMotorFront.configFactoryDefault();

    //Setup the integrated sensor
    MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    //Slave the front motors to their respective back motors
    MainLeftMotorFront.follow(MainLeftMotorBack);
    MainRightMotorFront.follow(MainRightMotorBack);

    //Disable voltage compensation, it's bad to be compensating voltage for a system which draws loads of amps
    MainLeftMotorBack.enableVoltageCompensation(false);
    MainLeftMotorFront.enableVoltageCompensation(false);
    MainRightMotorBack.enableVoltageCompensation(false);
    MainRightMotorFront.enableVoltageCompensation(false);

    //Have the back motors be on brake as suggested by Johnny
    MainLeftMotorBack.setNeutralMode(NeutralMode.Coast); //brake
    MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
    MainRightMotorBack.setNeutralMode(NeutralMode.Coast); //brake
    MainRightMotorFront.setNeutralMode(NeutralMode.Coast);

    //Invert one of the sides
    MainLeftMotorBack.setInverted(true);
    MainRightMotorBack.setInverted(false);
    MainLeftMotorFront.setInverted(InvertType.FollowMaster);
    MainRightMotorFront.setInverted(InvertType.FollowMaster);

    //Initialize the drivetrain API logic to be used on the CAN devices
    robotDrive = new DifferentialDrive(MainLeftMotorBack, MainRightMotorBack);

    //Initialize the drivetrain API which simulates the voltage outputs on the given CAN devices assigned to the DifferentialDrive class

    // robotDriveSim = new DifferentialDrivetrainSim(LinearSystemId.identifyVelocitySystem(Constants.kVLinear, Constants.kALinear), 
    // DCMotor.getFalcon500(2),
    // Constants.GEARING_drivetrainGearbox, 
    // Constants.trackWidth, 
    // Constants.drivetrainWheelRadius, 
    // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    
    robotDriveSim = new DifferentialDrivetrainSim(
      DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
      Constants.GEARING_drivetrainGearbox,               //Standard AndyMark Gearing reduction.
      2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
      26.5,                     //Mass of the robot is 26.5 kg.
      Constants.drivetrainWheelRadius,  //Robot uses 3" radius (6" diameter) wheels.
      Constants.trackWidth,                    //Distance between wheels is _ meters.
      
      /*
       * The standard deviations for measurement noise:
       * x and y:          0.001 m
       * heading:          0.001 rad
       * l and r velocity: 0.1   m/s
       * l and r position: 0.005 m
       */
      null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
    );

    SmartDashboard.putData("Field", field); //send field to NT

    robotDrive.setDeadband(Constants.driverJoystickDeadband); //Apply deadbanding to fix controller drift

    leftMotorSim = MainLeftMotorBack.getSimCollection();
    rightMotorSim = MainRightMotorBack.getSimCollection();

    limelightSim = new LimeLight(field);
    turretDirection = new LineRenderer(odometry.getEstimatedPosition().getX(), odometry.getEstimatedPosition().getY(), 3, 3, field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Invert the left input due to PS4 API thinking up is -1

    double input = ComputerVisionUtil.calculateDistanceToTarget(Constants.limelightHeight, 
    Constants.hubHeight, Units.degreesToRadians(Constants.limelightMountAngle), 
    Units.degreesToRadians(LimeLight.getTy()), Units.degreesToRadians(-LimeLight.getTx()));
    SmartDashboard.putNumber("distance to hub", input);
    
    robotDrive.arcadeDrive(-RobotContainer.driverJoystick.getRawAxis(1)*Constants.drivetrainSpeedLimiter, RobotContainer.driverJoystick.getRawAxis(2)*Constants.drivetrainSpeedLimiter);

    double leftSpeedMs = GearUtil.EncoderTicksPer100msToMetersPerSecond(MainLeftMotorBack.getSelectedSensorVelocity(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius);
    double rightSpeedMs = GearUtil.EncoderTicksPer100msToMetersPerSecond(MainLeftMotorBack.getSelectedSensorVelocity(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius);

    odometry.update(gyro.getRotation2d(), 
    new DifferentialDriveWheelSpeeds(leftSpeedMs, rightSpeedMs),
    GearUtil.encoderTicksToMeters(MainLeftMotorBack.getSelectedSensorPosition(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius),
    GearUtil.encoderTicksToMeters(MainRightMotorBack.getSelectedSensorPosition(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    Pose2d robotPose = odometry.getEstimatedPosition();
    field.setRobotPose(robotPose);
    double radian = Units.degreesToRadians(RobotContainer.shooterSubsystem.degreesOnTurret() - gyro.getAngle()); //turret is fixed to robot rotation however gyro is inverted
    Translation2d originOrientation = new Translation2d(Math.cos(radian), Math.sin(radian));
    Translation2d localPosition = robotPose.getTranslation().plus(originOrientation);
    Translation2d lookDirection = robotPose.getTranslation().minus(localPosition);
    Rotation2d look = new Rotation2d(lookDirection.getX(), lookDirection.getY());
    limelightSim.render(new Pose2d(localPosition, look));
    turretDirection.update(robotPose.getX(), robotPose.getY(), Constants.hubPosition.getX(), Constants.hubPosition.getY(), field);

    SmartDashboard.putNumber("distanceSim", limelightSim.getSimDistanceToHub());


    //For the motor master which is inverted, you'll need to invert it manually (ie with a negative sign) here when fetching any data 
    //CTRE doesn't support setInverted() for simulation
    
    leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    robotDriveSim.setInputs(leftMotorSim.getMotorOutputLeadVoltage(),
                         -rightMotorSim.getMotorOutputLeadVoltage());
    //The roboRIO updates at 50hz so you want to match what it actually is in simulation to get accurate simulations
    robotDriveSim.update(0.02);

    //Update sensors
    leftMotorSim.setIntegratedSensorRawPosition((int)GearUtil.metersToEncoderTicks(robotDriveSim.getLeftPositionMeters(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));
    leftMotorSim.setIntegratedSensorVelocity((int)GearUtil.metersPerSecondToEncoderTicksPer100ms(robotDriveSim.getLeftVelocityMetersPerSecond(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));
    rightMotorSim.setIntegratedSensorRawPosition((int)GearUtil.metersToEncoderTicks(-robotDriveSim.getRightPositionMeters(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));
    rightMotorSim.setIntegratedSensorVelocity((int)GearUtil.metersPerSecondToEncoderTicksPer100ms(-robotDriveSim.getRightVelocityMetersPerSecond(), Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));

    //Update simulation gyro, it's detached from the actual gyro
    gyroSim.setAngle(robotDriveSim.getHeading().getDegrees());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(GearUtil.EncoderTicksPer100msToMetersPerSecond(MainLeftMotorBack.getSelectedSensorVelocity(), 
    Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius), 
    GearUtil.EncoderTicksPer100msToMetersPerSecond(MainRightMotorBack.getSelectedSensorVelocity(), 
    Constants.GEARING_drivetrainGearbox, 2048.0, Constants.drivetrainWheelRadius));
  }
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    MainLeftMotorBack.setVoltage(leftVolts);
    MainRightMotorBack.setVoltage(rightVolts);
    robotDrive.feed(); //feed watchdog to prevent error from clogging can bus
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
  public void resetEncoders() {
    MainLeftMotorBack.setSelectedSensorPosition(0);
    MainRightMotorBack.setSelectedSensorPosition(0);
  }
  
}
