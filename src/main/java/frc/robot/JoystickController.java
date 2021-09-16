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
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.*;
//import frc.robot.commands.TestCommands.*;
//import frc.robot.commands.TestCommands.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class JoystickController {
  	private final Joystick joystick;
	public static JoystickController MAIN_JOYSTICK = null;
  	public static JoystickController COPILOT_JOYSTICK = null;

	
	private static JoystickController generateMainJoystick(){
		final Joystick joystick = new Joystick(0);
		//setButtonPressBehavior(joystick, 5, new IntakeStartRollerSpeed(), new IntakeStop());		
		setButtonPressBehavior(joystick,2, new ClimbStart(),new ClimbStop());
		setButtonPressBehavior(joystick,3, new ClimbDown(),new ClimbStop());
		//600
		////setButtonPressBehavior(joystick, 6, new ShootingStartFiringTheLaser(100, 80), new ShooterStopHoldYourFire(0)); 
		//setButtonBehavior(joystick, 3, new ClimbPrepare(3000, 100));
		//setButtonPressBehavior(joystick, 4, new ClimbStart(), new ClimbStopWench());   
		//setButtonPressBehavior(joystick, 2, new ConveyorReverse(), new IntakeStop());
		
	
		//setButtonBehavior(joystick, 3, new TestIntakeArm());
		
		//600,20
		//setButtonPressBehavior(joystick, 2, new ShootingStartFiringTheLaser(50, 50), new ShooterStopHoldYourFire(0));
		//setButtonPressBehavior(joystick, 3, new IntakeReverse(), new IntakeStopGERRevertToZeroRoller());	

		//setButtonPressBehavior(joystick, 2, new ClimbDown(), new ClimbStop());
		//setButtonBehavior(joystick, 7, new ServosMove(1));
		//setButtonBehavior(joystick, 2, new TestShooterPEWPEW());
		//setButtonPressBehavior(joystick, 3, new TestConveyor(), new Convey());	
		//setButtonPressBehavior(joystick, 2, new Convey(), new IntakeStopGERRevertToZeroRoller());	
		//setButtonBehavior(joystick, 4, new TestIntakeArm());

		

		return new JoystickController(joystick);
	}

	private static JoystickController generateCoPilotJoystick(){
		final Joystick joystick = new Joystick(1);

		/*setButtonPressBehavior(joystick, 5, new IntakeStartRollerSpeed(), new IntakeStop());		
		
		
		//600
		setButtonPressBehavior(joystick, 6, new ShootingStartFiringTheLaser(100, 80), new ShooterStopHoldYourFire(0)); 
		setButtonBehavior(joystick, 3, new ClimbPrepare(3000, 100));
		setButtonPressBehavior(joystick, 4, new ClimbStart(), new ClimbStopWench());   
		setButtonPressBehavior(joystick, 2, new ConveyorReverse(), new IntakeStop());
		

		*/
		return new JoystickController(joystick);
	}
	public static void Init(){
		MAIN_JOYSTICK =generateMainJoystick();
		COPILOT_JOYSTICK = generateCoPilotJoystick();
	}
	private static void setButtonBehavior(final Joystick joystick, final int buttonNumber, final Command whileHeldCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whileHeldCommand);
	}
	
	private static void setButtonHeldBehavior(final Joystick joystick, final int buttonNumber, 
										  final Command whileHeldCommand, final Command whenReleasedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whileHeld(whileHeldCommand);
		button.whenReleased(whenReleasedCommand);
	}
	private static void setButtonPressBehavior(final Joystick joystick, final int buttonNumber, final Command whenPressedCommand, final Command whenReleasedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whenPressedCommand);
		button.whenReleased(whenReleasedCommand);
	}

	private static void setButtonPressBehaviorSecondary(final Joystick joystick, final int buttonNumber, final Command whenPressedCommand) {
		final Button button = new JoystickButton(joystick, buttonNumber);
		button.whenPressed(whenPressedCommand);
	}
	
	/*8private static void setTriggerBehavior(final Joystick joystick, final int axis, final Command whenPressedCommand) {
		final Triggers trigger = new Triggers(joystick, axis);
		trigger.whenActive(whenPressedCommand);
	}
	private static void setTriggerBehavior(final Joystick joystick, final int axis, final Command whenPressedCommand, final Command whenReleasedCommand) {	
		final Triggers trigger = new Triggers(joystick, axis);
		trigger.whenActive(whenPressedCommand);
		trigger.whenInactive(whenReleasedCommand);*/
		
	//}
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
		return this.joystick.getRawAxis(3);
	}
	
	public double getRightStickX() {
		return this.joystick.getRawAxis(2);
	}
	
	public double getRightStickY() {
		return this.joystick.getRawAxis(5);
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
