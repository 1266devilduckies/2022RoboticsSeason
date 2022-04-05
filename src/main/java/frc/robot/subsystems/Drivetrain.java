package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  private final DifferentialDrive robotDrive;
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
  private final ADXRS450_Gyro gyro;

  DifferentialDriveOdometry odometry;
  
  private Field2d field = new Field2d(); //used to simulate the field for the simulated robot

  public Drivetrain() {
    MainLeftMotorBack = new WPI_TalonFX(Constants.CANID_mainLeftMotorBack);
    MainRightMotorBack = new WPI_TalonFX(Constants.CANID_mainRightMotorBack);
    MainLeftMotorFront = new WPI_TalonFX(Constants.CANID_mainLeftMotorFront);
    MainRightMotorFront = new WPI_TalonFX(Constants.CANID_mainRightMotorFront);

    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    gyroSim = new ADXRS450_GyroSim(gyro);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getAngle()), new Pose2d(5.0, 8.0, new Rotation2d()));
    
    MainLeftMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    MainRightMotorBack.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    //reset master encoders
    MainLeftMotorBack.setSelectedSensorPosition(0);
    MainRightMotorBack.setSelectedSensorPosition(0);

    //Slave the front motors to their respective back motors
    MainLeftMotorFront.follow(MainLeftMotorBack);
    MainRightMotorFront.follow(MainRightMotorBack);

    //Disable voltage compensation, it's bad to be compensating voltage for a system which draws loads of amps
    MainLeftMotorBack.enableVoltageCompensation(false);
    MainLeftMotorFront.enableVoltageCompensation(false);
    MainRightMotorBack.enableVoltageCompensation(false);
    MainRightMotorFront.enableVoltageCompensation(false);

    //Have the back motors be on brake as suggested by Johnny
    MainLeftMotorBack.setNeutralMode(NeutralMode.Brake);
    MainLeftMotorFront.setNeutralMode(NeutralMode.Coast);
    MainRightMotorBack.setNeutralMode(NeutralMode.Brake);
    MainRightMotorFront.setNeutralMode(NeutralMode.Coast);

    //Invert the left side to prevent the drivetrain from fighting itself
    MainLeftMotorFront.setInverted(true);
    MainLeftMotorBack.setInverted(true);

    //Initialize the drivetrain API logic to be used on the CAN devices
    robotDrive = new DifferentialDrive(MainLeftMotorBack, MainRightMotorBack);

    //Initialize the drivetrain API which simulates the voltage outputs on the given CAN devices assigned to the DifferentialDrive class
    robotDriveSim = new DifferentialDrivetrainSim(
      LinearSystemId.identifyDrivetrainSystem(Constants.kVLinear, Constants.kALinear, Constants.kVAngular, Constants.kAAngular),
      DCMotor.getFalcon500(2),
      Constants.GEARING_drivetrainGearbox,
      Constants.trackWidth,
      Constants.drivetrainWheelRadius,
      // The standard deviations for measurement noise:
      // x and y:          0.001 m
      // heading:          0.001 rad
      // l and r velocity: 0.1   m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    );

    SmartDashboard.putData("Field", field); //send field to NT

    robotDrive.setDeadband(Constants.driverJoystickDeadband); //Apply deadbanding to fix controller drift

    leftMotorSim = MainLeftMotorBack.getSimCollection();
    rightMotorSim = MainRightMotorBack.getSimCollection();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    robotDrive.arcadeDrive(RobotContainer.driverJoystick.getY()*Constants.drivetrainSpeedLimiter, RobotContainer.driverJoystick.getX()*Constants.drivetrainSpeedLimiter);

    odometry.update(gyro.getRotation2d(),
    (MainLeftMotorBack.getSelectedSensorPosition() / 2048.0) * (2*Math.PI*Constants.drivetrainWheelRadius), //encoder position in meters
    (MainRightMotorBack.getSelectedSensorPosition() / 2048.0) * (2*Math.PI*Constants.drivetrainWheelRadius)); //encoder position in meters
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    //Multiply the percent output by the input voltage to get the actual voltage on the motor
    robotDriveSim.setInputs(MainLeftMotorBack.get() * RobotController.getInputVoltage(), MainRightMotorBack.get() * RobotController.getInputVoltage());
    //The roboRIO updates at 50hz so you want to match what it actually is in simulation to get accurate simulations
    robotDriveSim.update(0.02);

    //Update sensors
    leftMotorSim.setIntegratedSensorRawPosition((int) (2048.0*((robotDriveSim.getLeftPositionMeters()) / (2*Math.PI*Constants.drivetrainWheelRadius)))); //conversion from meters to encoders
    leftMotorSim.setIntegratedSensorVelocity((int) (2048.0*((robotDriveSim.getLeftVelocityMetersPerSecond()/10.0) / (2*Math.PI*Constants.drivetrainWheelRadius)))); //conversion from m/s to encoder ticks/100ms
    rightMotorSim.setIntegratedSensorRawPosition((int) (2048.0*((robotDriveSim.getRightPositionMeters()) / (2*Math.PI*Constants.drivetrainWheelRadius)))); //conversion from meters to encoders
    rightMotorSim.setIntegratedSensorVelocity((int) (2048.0*((robotDriveSim.getRightVelocityMetersPerSecond()/10.0) / (2*Math.PI*Constants.drivetrainWheelRadius)))); //conversion from m/s to encoder ticks/100ms

    gyroSim.setAngle(-robotDriveSim.getHeading().getDegrees());
  }
}
