/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.XboxController;

public class JoystickController {
  	private final Joystick joystick;
	public static JoystickController MAIN_JOYSTICK;
  	public static JoystickController COPILOT_JOYSTICK;

	
	private static JoystickController generateMainJoystick(){
		final Joystick joystick = new Joystick(0);
	
		return new JoystickController(joystick);
		
		
		
	}//JoystickController generateMainJoystick()

	private static JoystickController generateCoPilotJoystick(){
		final Joystick joystick = new Joystick(1);
		setPOVButtonBehavior(joystick, 0, new ReverseIntake(), new GoBackNormalReverseIntake());
		setButtonHeldBehavior(joystick, 8, new PewPewStart(), null);
		setButtonHeldBehavior(joystick, 4, new ReverseIntake(), new GoBackNormalReverseIntake());
		setButtonHeldBehavior(joystick, 1, new IntakeDownDemo(), null);
		setButtonHeldBehavior(joystick, 3, new IntakeUpDemo(), null);
		return new JoystickController(joystick);
	} 
	public static void Init(){
		MAIN_JOYSTICK = generateMainJoystick();
		COPILOT_JOYSTICK = generateCoPilotJoystick();
	}
	private static void setButtonBehavior(final Joystick joystick, final int buttonNumber, final Command whileHeldCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whileHeldCommand);
	}

	private static void setPOVButtonBehavior(final Joystick joystick, final int angle, 
			final Command whileHeldCommand, final Command whenReleasedCommand) {
		final POVButton povButton = new POVButton(joystick, angle);
		if (whileHeldCommand != null) {
			povButton.whileHeld(whileHeldCommand);
			}
		if (whenReleasedCommand != null) {
			povButton.whenReleased(whenReleasedCommand);
		}
	}
	
	private static void setButtonHeldBehavior
	(final Joystick joystick, final int buttonNumber, 
										  final Command whileHeldCommand, final Command whenReleasedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		if (whileHeldCommand != null) {
		button.whileHeld(whileHeldCommand);
		}
		if (whenReleasedCommand != null) {
		button.whenReleased(whenReleasedCommand);
		}
	}
	private static void setButtonPressBehavior(final Joystick joystick, final int buttonNumber, final Command whenPressedCommand, final Command whenReleasedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		if (whenPressedCommand != null) {
		button.whenPressed(whenPressedCommand);
		}
		if (whenReleasedCommand != null) {
		button.whenReleased(whenReleasedCommand);
		}
	}
	/*
	private static void setButtonPressBehaviorSecondary(final Joystick joystick, final int buttonNumber, final Command whenPressedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whenPressedCommand);
	}*/
	JoystickController(final Joystick joystick){
		this.joystick = joystick;
	}
	
	public double getLeftStickX(){
		return this.joystick.getRawAxis(0);
	}
	
	public double getLeftStickY(){
		return this.joystick.getRawAxis(1);
	}
	
	public double getLeftTrigger(){
		return this.joystick.getRawAxis(2);
	}
	
	public double getRightTrigger(){
		return this.joystick.getRawAxis(5);
	}
	
	public double getRightStickX() {
		return this.joystick.getRawAxis(2);
	}
	
	public double getRightStickY() {
		return this.joystick.getRawAxis(3);
	}

	public boolean RightTriggeredPressed() {
		if (this.joystick.getRawAxis(3) > 0) {
			return true;
		}
		else {
			return false;
		}
	}
	
	public boolean LeftTriggeredPressed() {
		if (this.joystick.getRawAxis(2) > .25) {
			return true;
		}
		else {
			return false;
		}
	}

	public double GetPov() {
		double povValue = this.joystick.getPOV();
		return povValue;
	}
}
