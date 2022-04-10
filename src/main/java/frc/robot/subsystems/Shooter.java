package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX leftFlywheelMotor;
  private final TalonFXSimCollection leftFlywheelMotorSim;
  private final WPI_TalonFX rightFlywheelMotor;
  private final TalonFXSimCollection rightFlywheelMotorSim;
  private final WPI_TalonFX turretAlignmentMotor;
  private final TalonFXSimCollection turretAlignmentMotorSim;
  private final WPI_VictorSPX indexerMotor;
  private final VictorSPXSimCollection indexerMotorSim;
  private final PIDController turretAlignmentPIDController;
  private double flywheelTargetRPM = 0.0;
  private NetworkTable limelightTable;
  private double dx = 0.0;
  private double dy = 0.0;
  private double canSeeAnyTarget = 0.0;

  public Shooter() {
    leftFlywheelMotor = new WPI_TalonFX(Constants.CANID_leftFlywheelMotor);
    rightFlywheelMotor = new WPI_TalonFX(Constants.CANID_rightFlywheelMotor);
    indexerMotor = new WPI_VictorSPX(Constants.CANID_indexerMotor);
    turretAlignmentMotor = new WPI_TalonFX(Constants.CANID_turretAlignmentMotor);
    
    //config flywheel motors
    leftFlywheelMotor.configFactoryDefault();
    rightFlywheelMotor.configFactoryDefault();

    leftFlywheelMotor.setNeutralMode(NeutralMode.Coast);
    rightFlywheelMotor.setNeutralMode(NeutralMode.Coast);

    leftFlywheelMotor.setInverted(false);
    rightFlywheelMotor.setInverted(InvertType.OpposeMaster);

    leftFlywheelMotor.config_kP(0, Constants.PID_kP_flywheel);
    leftFlywheelMotor.config_kF(0, Constants.PID_kF_flywheel);

    leftFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    rightFlywheelMotor.follow(leftFlywheelMotor);

    //config turret alignment motor
    turretAlignmentMotor.configFactoryDefault();

    turretAlignmentMotor.setNeutralMode(NeutralMode.Brake);

    turretAlignmentMotor.setInverted(false);

    turretAlignmentPIDController = new PIDController(Constants.PID_kP_turretAlignment, Constants.PID_kI_turretAlignment, Constants.PID_kD_turretAlignment);
    turretAlignmentPIDController.setTolerance(2,15); //returns true if we are within 2 degrees of 0 degrees AND the rate of change currently (derivative) is less than 15 degrees
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //bind all of the simulated motors
    leftFlywheelMotorSim = leftFlywheelMotor.getSimCollection();
    rightFlywheelMotorSim = rightFlywheelMotor.getSimCollection();
    turretAlignmentMotorSim = turretAlignmentMotor.getSimCollection();
    indexerMotorSim = indexerMotor.getSimCollection();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    dx = limelightTable.getEntry("tx").getDouble(0.0);
    dy = limelightTable.getEntry("ty").getDouble(0.0);
    canSeeAnyTarget = limelightTable.getEntry("tv").getDouble(0.0);

    if (canSeeAnyTarget == 1.0 && !turretAlignmentPIDController.atSetpoint()) {
      turretAlignmentMotor.set(turretAlignmentPIDController.calculate(dx, 0.0));
    } else {
      turretAlignmentMotor.set(0.0);
    }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    leftFlywheelMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rightFlywheelMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    turretAlignmentMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    indexerMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
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
  public boolean alignedToGoalpost() {
    return canSeeAnyTarget == 1.0 && turretAlignmentPIDController.atSetpoint();
  }
  public double getCurrentRPM() {
    return RobotContainer.EncoderTicksPer100msToRPM(leftFlywheelMotor.getSelectedSensorVelocity(), Constants.GEARING_drivetrainGearbox, 2048.0);
  }
  public void setRPM(double rpm) {
    leftFlywheelMotor.set(ControlMode.Velocity, RobotContainer.RPMToEncoderTicksPer100ms(rpm, Constants.GEARING_drivetrainGearbox, 2048.0));
  }

  public void setToCoast() {
    leftFlywheelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetEncoders() {
    leftFlywheelMotor.setSelectedSensorPosition(0);
  }
}
