package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
    private final WPI_TalonFX leftClimberMotor;
    private final WPI_TalonFX rightClimberMotor;

    private final TalonFXSimCollection leftClimberMotorSim;
    private final TalonFXSimCollection rightClimberMotorSim;
    public Climber() {
        leftClimberMotor = new WPI_TalonFX(Constants.CANID_leftClimberMotor);
        rightClimberMotor = new WPI_TalonFX(Constants.CANID_rightClimberMotor);

        leftClimberMotor.configForwardSoftLimitThreshold(Constants.lowerBoundClimber, 0);
        leftClimberMotor.configReverseSoftLimitThreshold(Constants.upperBoundClimber, 0);
        leftClimberMotor.configForwardSoftLimitEnable(true, 0);
        leftClimberMotor.configReverseSoftLimitEnable(true, 0);
        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        leftClimberMotor.setInverted(true);

        rightClimberMotor.configForwardSoftLimitThreshold(Constants.lowerBoundClimber, 0);
        rightClimberMotor.configReverseSoftLimitThreshold(Constants.upperBoundClimber, 0);
        rightClimberMotor.configForwardSoftLimitEnable(true, 0);
        rightClimberMotor.configReverseSoftLimitEnable(true, 0);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setInverted(InvertType.OpposeMaster);

        rightClimberMotor.follow(leftClimberMotor);

        leftClimberMotorSim = leftClimberMotor.getSimCollection();
        rightClimberMotorSim = rightClimberMotor.getSimCollection();
    }

    //speed should be between -1 to 1 but it wont matter as it has a clamp function built-in
    public void setClimberPercentOutput(double speed) {
        leftClimberMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void periodic() {
        double lVal = -RobotContainer.operatorJoystick.getY();
        //deadband needs to be around half because the operators joystick is trash now
        double speed = 0.1;
        if (leftClimberMotor.getSelectedSensorPosition() < Constants.upperBoundClimber/2.0) {
            speed = 0.4;
        } else if (leftClimberMotor.getSelectedSensorPosition() < Constants.upperBoundClimber/8.0) {
            speed = 0.2;
        }
        if (lVal > 0.5) {
            setClimberPercentOutput(-speed);
        } else if (lVal < -0.5) {
            setClimberPercentOutput(speed);
        } else {
            setClimberPercentOutput(0.);
        }
    }
     public void simulationPeriodic() {
        leftClimberMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        rightClimberMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
     }
     public void resetEncoders() {
         leftClimberMotor.setSelectedSensorPosition(0);
         rightClimberMotor.setSelectedSensorPosition(0);
     }
}
