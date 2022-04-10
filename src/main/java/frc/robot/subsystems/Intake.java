package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final Solenoid singleSolenoid;
  private final Compressor pcmCompressor;
  private final WPI_VictorSPX intakeMotor;
  private final VictorSPXSimCollection intakeMotorSim;

  public Intake() {
    singleSolenoid = new Solenoid(Constants.CANID_pneumaticsControlModule, PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidPort);
    pcmCompressor = new Compressor(Constants.CANID_pneumaticsControlModule, PneumaticsModuleType.CTREPCM);
    intakeMotor = new WPI_VictorSPX(Constants.CANID_intakeMotor);

    //configure motor
    intakeMotor.configFactoryDefault();
    intakeMotor.enableVoltageCompensation(false);
    intakeMotor.setInverted(false);

    intakeMotorSim = intakeMotor.getSimCollection();
    pcmCompressor.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    intakeMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
  }
  public void setSolenoid(boolean mode) {
    singleSolenoid.set(mode);
  }

  public void setIntakeMotor(double percentOutput) {
    intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }
}
