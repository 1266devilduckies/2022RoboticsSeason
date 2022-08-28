package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GearUtil;
import frc.robot.LimeLight;
import frc.robot.VectorUtil;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftFlywheelMotor;
  private final TalonFXSimCollection leftFlywheelMotorSim;
  private final WPI_TalonFX rightFlywheelMotor;
  private final TalonFXSimCollection rightFlywheelMotorSim;
  public WPI_TalonFX turretAlignmentMotor;
  private final TalonFXSimCollection turretAlignmentMotorSim;
  private final WPI_VictorSPX indexerMotor;
  private final VictorSPXSimCollection indexerMotorSim;
  private final FlywheelSim flywheelSim;
  private final SingleJointedArmSim turretSim;

  private double canSeeAnyTarget = 0.0;
  private boolean aligned = false;
  public static boolean startedSeeking = false;

  public Shooter() {
    leftFlywheelMotor = new WPI_TalonFX(Constants.CANID_leftFlywheelMotor);
    rightFlywheelMotor = new WPI_TalonFX(Constants.CANID_rightFlywheelMotor);
    indexerMotor = new WPI_VictorSPX(Constants.CANID_indexerMotor);
    turretAlignmentMotor = new WPI_TalonFX(Constants.CANID_turretAlignmentMotor);
    
    indexerMotor.configFactoryDefault();
    indexerMotor.setNeutralMode(NeutralMode.Brake);
    indexerMotor.setInverted(false);

    //config flywheel motors
    leftFlywheelMotor.configFactoryDefault();
    rightFlywheelMotor.configFactoryDefault();

    leftFlywheelMotor.setNeutralMode(NeutralMode.Coast);
    rightFlywheelMotor.setNeutralMode(NeutralMode.Coast);

    leftFlywheelMotor.setInverted(false);
    rightFlywheelMotor.setInverted(InvertType.OpposeMaster);

    leftFlywheelMotor.config_kP(0, Constants.PID_kP_flywheel);

    leftFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    rightFlywheelMotor.follow(leftFlywheelMotor);

    //config turret alignment motor
    turretAlignmentMotor.configFactoryDefault();

    turretAlignmentMotor.setNeutralMode(NeutralMode.Brake);

    turretAlignmentMotor.setInverted(false);

    turretAlignmentMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    
    turretAlignmentMotor.configForwardSoftLimitThreshold(Constants.upperBoundTicks, 0);
    turretAlignmentMotor.configReverseSoftLimitThreshold(Constants.lowerBoundTicks, 0);
    turretAlignmentMotor.configForwardSoftLimitEnable(true, 0);
    turretAlignmentMotor.configReverseSoftLimitEnable(true, 0);

    turretAlignmentMotor.config_kP(0, Constants.PID_kP_turretAlignment);
    turretAlignmentMotor.config_kD(0, Constants.PID_kD_turretAlignment);

    turretAlignmentMotor.configMotionCruiseVelocity(GearUtil.RPMToEncoderTicksPer100ms(220, Constants.GEARING_turret, 2048.));
    turretAlignmentMotor.configMotionAcceleration(GearUtil.RPMToEncoderTicksPer100ms(220, Constants.GEARING_turret, 2048.));
    turretAlignmentMotor.configMotionSCurveStrength(0);

    //bind all of the simulated motors
    leftFlywheelMotorSim = leftFlywheelMotor.getSimCollection();
    rightFlywheelMotorSim = rightFlywheelMotor.getSimCollection();
    turretAlignmentMotorSim = turretAlignmentMotor.getSimCollection();
    indexerMotorSim = indexerMotor.getSimCollection();

    flywheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.kVFlywheel / 6.28, Constants.kAFlywheel / 6.28), DCMotor.getFalcon500(2), 1.0);
    turretSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), Constants.GEARING_turret, SingleJointedArmSim.estimateMOI(.3, 5), .3, 
    Units.degreesToRadians(Constants.lowerBoundShooterDegrees), 
    Units.degreesToRadians(Constants.upperBoundShooterDegrees),
    5, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    canSeeAnyTarget = LimeLight.getTv();
    double rotationSetpoint = 0; //in terms of degrees
    double rotation = turretAlignmentMotor.getSelectedSensorPosition()/Constants.ticksPerDegreeTurret; //in terms of degrees relative to turret
    if (canSeeAnyTarget == 1.0) {
      rotationSetpoint = rotation + LimeLight.getTx(); //get offset based on camera
      Object[] visionData = LimeLight.getRobotPoseFromVision();

      if (!aligned) {
      Drivetrain.odometry.addVisionMeasurement((Pose2d)LimeLight.getRobotPoseFromVision()[0], Timer.getFPGATimestamp());
      feedCLRotateToAngle(rotationSetpoint);
      }
      SmartDashboard.putNumber("distance ft", (double)visionData[1]);
      SmartDashboard.putBoolean("using odometry for turret tracking", false);
    }
    if (canSeeAnyTarget == 0.0) {
      Pose2d odometryPose = Drivetrain.odometry.getEstimatedPosition();
      Translation2d robotToHub = Constants.hubPosition.minus(odometryPose.getTranslation()); //displacement vector robot to hub
      Translation2d lookVectorDirection = new Translation2d(odometryPose.getRotation().getCos(), odometryPose.getRotation().getSin());
      Translation2d lookVectorOnRobot = odometryPose.getTranslation().plus(lookVectorDirection);
      double angleDifferenceHeadingToHub = VectorUtil.dot(VectorUtil.unit(robotToHub), VectorUtil.unit(lookVectorOnRobot));

      rotationSetpoint = (rotation - angleDifferenceHeadingToHub) + rotation;
      feedCLRotateToAngle(rotationSetpoint);
      turretAlignmentMotor.set(ControlMode.PercentOutput, rotationSetpoint*Constants.ticksPerDegreeTurret);
      SmartDashboard.putNumber("distance ft", robotToHub.getNorm()); //use odometry data
      SmartDashboard.putBoolean("using odometry for turret tracking", true);
    }
    aligned = Math.abs(turretAlignmentMotor.getSelectedSensorPosition() - rotationSetpoint*Constants.ticksPerDegreeTurret) < Constants.tickTolerance; //trusts odometry and camera
    SmartDashboard.putNumber("tx", LimeLight.getTx());
    SmartDashboard.putBoolean("Ready To Shoot", aligned);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    leftFlywheelMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightFlywheelMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    turretAlignmentMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    indexerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    
    flywheelSim.setInputVoltage(leftFlywheelMotor.getMotorOutputVoltage());
    flywheelSim.update(0.02);
    turretSim.setInputVoltage(turretAlignmentMotor.getMotorOutputVoltage());
    turretSim.update(0.02);

    leftFlywheelMotorSim.setIntegratedSensorVelocity((int)GearUtil.RPMToEncoderTicksPer100ms(flywheelSim.getAngularVelocityRPM(), 1.0, 2048.0));
    rightFlywheelMotorSim.setIntegratedSensorVelocity((int)GearUtil.RPMToEncoderTicksPer100ms(flywheelSim.getAngularVelocityRPM(), 1.0, 2048.0));

    turretAlignmentMotorSim.setIntegratedSensorRawPosition((int)(Units.radiansToDegrees(turretSim.getAngleRads())*Constants.ticksPerDegreeTurret));
  }

  public void setMasterMotorOnFlywheel(double percentOutput) {
    leftFlywheelMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  public boolean shooterIsAligned() {
    return aligned;
  }
  public double getCurrentRPM() {
    return GearUtil.EncoderTicksPer100msToRPM(leftFlywheelMotor.getSelectedSensorVelocity(), 1.0, 2048.0);
  }
  public void setRPM(double rpm) {
    leftFlywheelMotor.set(ControlMode.Velocity, 
    GearUtil.RPMToEncoderTicksPer100ms(rpm, 1.0, 2048.0), 
    DemandType.ArbitraryFeedForward, 
    Constants.SIMPLE_MOTOR_FEEDFORWARD_flywheel.calculate(rpm / 60.) / RobotController.getBatteryVoltage());
  }

  public void setToCoast() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetEncoders() {
    leftFlywheelMotor.setSelectedSensorPosition(0);
  }

  public void setIndexerMotor(double percentOutput) {
    indexerMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  public double degreesOnTurret() {
    return turretAlignmentMotor.getSelectedSensorPosition()/((double)Constants.ticksPerDegreeTurret);
  }
  public double getTurretPosition() {
    return turretAlignmentMotor.getSelectedSensorPosition() / Constants.GEARING_turret; 
  }
  public boolean isAtLowerBound(double ticks) {
    return Math.abs(ticks-Constants.lowerBoundTicks) < Constants.tickTolerance;
  }
  public boolean isAtUpperBound(double ticks) {
    return Math.abs(ticks-Constants.upperBoundTicks) < Constants.tickTolerance;
  }
  private void feedCLRotateToAngle(double degrees) {
    double newAngle = (degrees + 170) % 360 - 170;
    if (newAngle - 360 > degreesOnTurret()) {
      newAngle -= 360;
    }
    turretAlignmentMotor.set(ControlMode.MotionMagic, newAngle*Constants.ticksPerDegreeTurret);
  }
}
