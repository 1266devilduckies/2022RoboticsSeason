/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.*;

public class JoystickController {
	private final Joystick joystick;
	public static JoystickController MAIN_JOYSTICK;
	public static JoystickController COPILOT_JOYSTICK;

	// creates Main joystick object
	private static JoystickController generateMainJoystick() {
		final Joystick joystick = new Joystick(0);

		return new JoystickController(joystick);

	}

	// create CoPilot joystick object
	private static JoystickController generateCoPilotJoystick() {
		final Joystick joystick = new Joystick(1);
		setPOVButtonBehavior(joystick, 0, new ReverseIntake(), new GoBackNormalReverseIntake());
		setButtonHeldBehavior(joystick, 9, new SlowShot(), null);
		setButtonHeldBehavior(joystick, 8, new PewPewStart(), null);
		setButtonHeldBehavior(joystick, 4, new AlignToTarget(), null);
		setButtonHeldBehavior(joystick, 1, new IntakeDown(), new IntakeUp());
		setButtonHeldBehavior(joystick, 2, new Climbstart(), new Climberstop());
		setButtonHeldBehavior(joystick, 5, new climbreverse(), new Climberstop());
		setButtonHeldBehavior(joystick, 6, new climbpart2(), new climbpart2stop());
		return new JoystickController(joystick);
	}

	// on init creates joysticks
	public static void Init() {
		MAIN_JOYSTICK = generateMainJoystick();
		COPILOT_JOYSTICK = generateCoPilotJoystick();
	}

	// button when pressed run Command
	private static void setButtonBehavior(final Joystick joystick, final int buttonNumber,
			final Command whileHeldCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whileHeldCommand);
	}

	// DPad when held run Command, when released run other Command
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

	// button when held run Command, when released run other Command
	private static void setButtonHeldBehavior(final Joystick joystick, final int buttonNumber,
			final Command whileHeldCommand, final Command whenReleasedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		if (whileHeldCommand != null) {
			button.whileHeld(whileHeldCommand);
		}
		if (whenReleasedCommand != null) {
			button.whenReleased(whenReleasedCommand);
		}
	}

	JoystickController(final Joystick joystick) {
		this.joystick = joystick;
	}

	// Joystick getters
    // change when we get new contollers por favor - Benny
	public double getLeftStickX() {
		return this.joystick.getRawAxis(0);
	}

	public double getLeftStickY() {
		return this.joystick.getRawAxis(1);
	}

	/*public double getLeftTrigger() {
		return this.joystick.getRawAxis(2);
	}

	public double getRightTrigger() {
		return this.joystick.getRawAxis(5);
	}*/

	public double getRightStickX() {
		return this.joystick.getRawAxis(4);
	}

	public double getRightStickY() {
		return this.joystick.getRawAxis(5);
	}

	public boolean RightTriggeredPressed() {
		if (this.joystick.getRawAxis(3) > 0) {
			return true;
		} else {
			return false;
		}
	}

	public boolean LeftTriggeredPressed() {
		if (this.joystick.getRawAxis(2) > .25) {
			return true;
		} else {
			return false;
		}
	}

	public double GetPov() {
		double povValue = this.joystick.getPOV();
		return povValue;
	}
}
