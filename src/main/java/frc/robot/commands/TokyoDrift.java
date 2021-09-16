/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.JoystickController;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;

public class TokyoDrift extends Command {
  /**
   * Creates a new TokyoDrift.
   */
    Drivetrain drivetrain = Robot.drivetrain;
    JoystickController mainJoystick = JoystickController.MAIN_JOYSTICK;
    JoystickController coPilotJoystick = JoystickController.COPILOT_JOYSTICK;
  public TokyoDrift() {
    requires(Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
	  //System.out.println("here");
      double mainY = mainJoystick.getLeftStickY();
    	double mainX = mainJoystick.getRightStickX();
    	//directional control for main
    	//double overdrive = mainJoystick.getRightTrigger();
    	//double underdrive = mainJoystick.getLeftTrigger();
    	//Variable to turn on overdrive
    	double coY = coPilotJoystick.getLeftStickY();
    	double coX = coPilotJoystick.getLeftStickX();
    	//direction control for co
    	//double takeover = coPilotJoystick.getRightTrigger();
    	double normalSpeed = 0.75;
    	double normalTurn = 0.75;
    	double underSpeed = 0.5;
    	double underTurn = 0.5;
    	double overSpeed = 1;
    	double overTurn = 0.9;
    	double coSpeed = 0.4;
    	double coTurn = 0.2;
    	//speed values ^
    	
    	//System.out.println("mainY" + mainY);
    	//System.out.println("mainX" + mainX);
    	
    	
    		drivetrain.arcadeDriveVoltage(-mainY, -mainX, normalTurn, normalSpeed);
    		//main driving - sets normal speed
    	
    	
    	
  }


  // Called once the command ends or is interrupted.
  @Override
  protected void end() {  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
