package frc.robot.commands;

import frc.robot.JoystickController;
import frc.robot.Robot;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotMap;

//import javax.management.modelmbean.RequiredModelMBean;
import frc.robot.EncoderSetter;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterKearnyDriving extends Command {
  // Drivetrain drivetrain = Robot.drivetrain;
  JoystickController mainJoystick = JoystickController.MAIN_JOYSTICK;
  JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;

  public BetterKearnyDriving() {
    // requires(Robot.drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double x = mainJoystick.getLeftStickY();
    double y = mainJoystick.getRightStickX();
    /*
     * double coY = coPilotJoystick.getLeftStickX();
     * double coX = coPilotJoystick.getLeftStickY();
     */

    // here's where we scale the speeds for the motors -JM
    double normalSpeed = -0.75;
    double normalTurn = -0.75;
    /*
     * double underSpeed = 0.375;
     * double underTurn = 0.375;
     * double overSpeed = 1;
     * double overTurn = 0.8;
     * double coSpeed = 0.3;
     * double coturn = 0.15;
     */

    // System.out.println("mainY = " + mainY);
    // System.out.println("mainX = " + mainX);
    Robot.drivetrain.arcadeDriveVoltage(x, y, -normalSpeed, normalTurn);
    EncoderSetter.updateEncoders();
    SmartDashboard.putNumber("Avg Position in Meters",
        EncoderSetter.nativeUnitsToDistanceMeters(RobotMap.avgPositionRaw));
    SmartDashboard.putNumber("Avg Position RAW", RobotMap.avgPositionRaw);

  }

  @Override
  protected void end() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}