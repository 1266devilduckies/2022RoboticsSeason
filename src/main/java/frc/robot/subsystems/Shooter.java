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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToTarget;

enum ShooterState {
  IDLE,
  ALIGN,
  SEARCH,
  SWEEPLEFT,
  SWEEPRIGHT
}
public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftFlywheelMotor;
  private final TalonFXSimCollection leftFlywheelMotorSim;
  private final WPI_TalonFX rightFlywheelMotor;
  private final TalonFXSimCollection rightFlywheelMotorSim;
  public final WPI_TalonFX turretAlignmentMotor;
  private final TalonFXSimCollection turretAlignmentMotorSim;
  private final WPI_VictorSPX indexerMotor;
  private final VictorSPXSimCollection indexerMotorSim;
  private final PIDController turretAlignmentPIDController;
  private final FlywheelSim flywheelSim;

  private double flywheelTargetRPM = 0.0;
  private NetworkTable limelightTable;
  private double canSeeAnyTarget = 0.0;
  private boolean canShoot = false;
  public boolean startedToBeAligned = false;
  

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
    turretAlignmentMotor.config_kI(0, Constants.PID_kI_turretAlignment);
    turretAlignmentMotor.config_kD(0, Constants.PID_kD_turretAlignment);

    turretAlignmentPIDController = new PIDController(Constants.PID_kP_turretAlignment, Constants.PID_kI_turretAlignment, Constants.PID_kD_turretAlignment);
    turretAlignmentPIDController.setTolerance(1,15); //returns true if we are within 1 degree of 0 degrees AND the rate of change currently (derivative) is less than 15 degrees
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //bind all of the simulated motors
    leftFlywheelMotorSim = leftFlywheelMotor.getSimCollection();
    rightFlywheelMotorSim = rightFlywheelMotor.getSimCollection();
    turretAlignmentMotorSim = turretAlignmentMotor.getSimCollection();
    indexerMotorSim = indexerMotor.getSimCollection();

    flywheelSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(Constants.kVFlywheel / 6.28, Constants.kAFlywheel / 6.28), DCMotor.getFalcon500(2), 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    canSeeAnyTarget = limelightTable.getEntry("tv").getDouble(0.0);

    if (canSeeAnyTarget == 1.0 && !startedToBeAligned) {
      startedToBeAligned = true;
      CommandScheduler.getInstance().schedule(new AlignToTarget(RobotContainer.shooterSubsystem));
    } 
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

    leftFlywheelMotorSim.setIntegratedSensorVelocity((int)RobotContainer.RPMToEncoderTicksPer100ms(flywheelSim.getAngularVelocityRPM(), 1.0, 2048.0));
    rightFlywheelMotorSim.setIntegratedSensorVelocity((int)RobotContainer.RPMToEncoderTicksPer100ms(flywheelSim.getAngularVelocityRPM(), 1.0, 2048.0));
    
  }

  public void setMasterMotorOnFlywheel(double percentOutput) {
    leftFlywheelMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  public void setTargetRPM(double newRPM) {
    flywheelTargetRPM = newRPM;
  }
  public double getTargetRPM() {
    return flywheelTargetRPM;
  }
  public boolean canShoot() {
    return this.canShoot;
  }
  public double getCurrentRPM() {
    return RobotContainer.EncoderTicksPer100msToRPM(leftFlywheelMotor.getSelectedSensorVelocity(), 1.0, 2048.0);
  }
  public void setRPM(double rpm) {
    leftFlywheelMotor.set(ControlMode.Velocity, 
    RobotContainer.RPMToEncoderTicksPer100ms(rpm, 1.0, 2048.0), 
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
    return indexerMotor.getSelectedSensorPosition()/((double)Constants.ticksPerDegreeTurret);
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
}
