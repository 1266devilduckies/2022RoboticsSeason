/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;

public class JoystickController {
	private final Joystick joystick;
	public static JoystickController MAIN_JOYSTICK;
	public static JoystickController COPILOT_JOYSTICK;
	private static final int pilotPort = 0;
	private static final int operatorPort = 1;
	// creates Main joystick object
	private static JoystickController generateMainJoystick() {
		final Joystick joystick = new Joystick(pilotPort);
		// this is for playstation
		setButtonHeldBehavior(joystick, 6, new StartIntake(), new StopIntake());
		setButtonHeldBehavior(joystick, 2, new SequentialCommandGroup(new AlignToTarget()).andThen(() -> RobotMap.pilotDisabled = false), null);
		//setButtonHeldBehavior(joystick, 3, new SequentialCommandGroup(new AlignToTarget(), new PositionRobotForShooter(), new PewPewStart(false, RobotMap.overrideVelocity)).andThen(() -> RobotMap.pilotDisabled = false), null);

		// this changes the direction of the intake motor while it is being held
		// it does not start it but rather it changes the polarity of the motor
		setPOVButtonBehavior(joystick, 0, new ReverseIntake(), new ReverseIntakeReleased());
		return new JoystickController(joystick);

	}

	// create CoPilot joystick object
	private static JoystickController generateCoPilotJoystick() {
		final Joystick joystick = new Joystick(operatorPort);
		// this is for xbox

		// goes for low ball shot
		setButtonHeldBehavior(joystick, 5, new SlowShot(), null);
		// goes for high ball shot
		setButtonHeldBehavior(joystick, 6, new PewPewStart(false), null);
		//setButtonPressBehavior(joystick, 2, new Climbstart());
		return new JoystickController(joystick);
	}

	// on init creates joysticks
	public static void Init() {
		MAIN_JOYSTICK = generateMainJoystick();
		COPILOT_JOYSTICK = generateCoPilotJoystick();
	}

	/*
	 * / button when pressed run Command
	 * private static void setButtonBehavior(final Joystick joystick, final int
	 * buttonNumber,
	 * final Command whileHeldCommand) {
	 * final Button button = new JoystickButton(joystick, buttonNumber);
	 * button.whenPressed(whileHeldCommand); // whenPressed
	 * }
	 */

	// DPad when held run Command, when released run other Command
	private static void setPOVButtonBehavior(final Joystick joystick, final int angle,
			final CommandBase whileHeldCommand, final CommandBase whenReleasedCommand) {
		final POVButton povButton = new POVButton(joystick, angle);
		if (whileHeldCommand != null) {
			povButton.whileHeld(whileHeldCommand);
		}
		if (whenReleasedCommand != null) {
			povButton.whenReleased(whenReleasedCommand);
		}
	}

private static void setButtonPressBehavior(final Joystick joystick, final int buttonNumber,
			final CommandBase whenPressedCommand) {
		final Button button = new JoystickButton(
				joystick, buttonNumber);
		if (whenPressedCommand != null) {
			button.whenPressed(whenPressedCommand);
		}
	}

	// button when held run Command, when released run other Command

	private static void setButtonHeldBehavior(final Joystick joystick, final int buttonNumber,
			final CommandBase whileHeldCommand, final CommandBase whenReleasedCommand) {
		final Button button = new JoystickButton(
				joystick, buttonNumber);
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
		double yAxis = 0.0;
		if (this.joystick.getPort() == pilotPort) {
			yAxis = this.joystick.getRawAxis(1);
		} else if (this.joystick.getPort() == operatorPort) {
			yAxis = this.joystick.getRawAxis(1);
		}
		return yAxis;
	}

	/*public double getLeftTrigger() {
		return this.joystick.getRawAxis(2);
	}*/

	/*public double getRightTrigger() {
		return this.joystick.getRawAxis(3);
	}*/

	public double getRightStickX() {
		return this.joystick.getRawAxis(2);
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
