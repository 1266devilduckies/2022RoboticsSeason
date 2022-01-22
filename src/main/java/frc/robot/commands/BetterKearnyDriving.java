package frc.robot.commands;
import frc.robot.JoystickController;
import frc.robot.Robot;
//import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotMap;

//import javax.management.modelmbean.RequiredModelMBean;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BetterKearnyDriving extends Command {
    //Drivetrain drivetrain = Robot.drivetrain;
    JoystickController mainJoystick = JoystickController.MAIN_JOYSTICK;
    JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK; 

  public BetterKearnyDriving(){
    //requires(Robot.drivetrain);  
  }

@Override
public void initialize(){
}

@Override
public void execute(){
  double x = mainJoystick.getLeftStickY();
  double y = mainJoystick.getRightStickX();
  /*double coY = coPilotJoystick.getLeftStickX();
  double coX = coPilotJoystick.getLeftStickY(); */

  //here's where we scale the speeds for the motors -JM
  double normalSpeed = -0.75;
  double normalTurn = -0.75;
  /*double underSpeed = 0.375;
  double underTurn = 0.375;
  double overSpeed = 1;
  double overTurn = 0.8;
  double coSpeed = 0.3;
  double coturn = 0.15;*/
  
  //System.out.println("mainY = " + mainY);
  //System.out.println("mainX = " + mainX);
  Robot.drivetrain.arcadeDriveVoltage(x,y,-normalSpeed,normalTurn);
  SmartDashboard.putNumber("LB Position", RobotMap.MainLeftMotorBack.getSelectedSensorPosition(0));
  SmartDashboard.putNumber("LF Position", RobotMap.MainLeftMotorFront.getSelectedSensorPosition(0));
  SmartDashboard.putNumber("RB Position", RobotMap.MainRightMotorBack.getSelectedSensorPosition(0));
  SmartDashboard.putNumber("RF Position", RobotMap.MainRightMotorFront.getSelectedSensorPosition(0));
  //it is updating the data
  SmartDashboard.putNumber("Time Alapsed", (double)(System.currentTimeMillis()));
}
@Override
protected void end() {
}
@Override
public boolean isFinished() {
    return false;
}
}