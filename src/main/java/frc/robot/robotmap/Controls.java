package frc.robot.robotmap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class Controls {

	// Buttons
	private  static final int aButtonIndex = 1;
	private static final int bButtonIndex = 2;
	private static final int yButtonIndex = 4;
	private static final int xButtonIndex = 3;
	private static final int backButtonIndex = 7;
	private static final int startButtonIndex = 8;
	private static final int leftBumperIndex = 5;
	private static final int rightBumperIndex = 6;
	private static final int leftButtonJoystickIndex = 9;
	private static final int rightButtonJoystickIndex = 10;

	// Joysticks and triggers
	private static final int rightTriggerIndex = 3;
	private static final int leftTriggerIndex = 2;
	private static final int leftStickXIndex = 0;
	public static final int leftStickYIndex = 1; // TODO Figure out how to make this private (used for indexer)
	private static final int rightStickXIndex = 4;
	private static final int rightStickYIndex = 5;

	//
	// Driver controls
	//

	// Driver controller
	public static final XboxController driverController = new XboxController(0);

	// Auto intake start
	public static final JoystickButton autoIntakeStartButton = new JoystickButton(driverController, aButtonIndex);

	// Robot orientation reset
	public static final JoystickButton resetRobotOrientation = new JoystickButton(driverController, backButtonIndex);

	// Drive slow button
	public static final JoystickButton driveSlowButton = new JoystickButton(driverController, leftButtonJoystickIndex);

	// Climb button
	public static final JoystickButton climbButton = new JoystickButton(driverController, xButtonIndex);


	//
	// Operator controls
	//

	// Operator controller
	public static final XboxController operatorController = new XboxController(1);

	// Wrong ball output button
	public static JoystickButton indexWrongBallOutButton = new JoystickButton(operatorController, leftBumperIndex);

	// Intake button
	public static JoystickButton intakeButton = new JoystickButton(operatorController, rightBumperIndex);

	// Shoot high af boiiiiiiiiiiiiiii
	public static JoystickButton shootHighAutomaticButton = new JoystickButton(operatorController, aButtonIndex);

}
