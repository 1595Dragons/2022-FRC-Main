// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Controllers
    public static final XboxController driver = new XboxController(0);
    public static final XboxController operator = new XboxController(1);

    // Buttons
    private static final int aButton = 1;
    private static final int bButton = 2;
    private static final int yButton = 4;
    private static final int xButton = 3;
    private static final int backButton = 7;
    private static final int startButton = 8;
    private static final int leftBumper = 5;
    private static final int rightBumper = 6;
    private static final int leftButtonJoystick = 9;
    private static final int rightButtonJoystick = 10;

    public static final JoystickButton resetRobotOrientation = new JoystickButton(driver, backButton);
    public static final JoystickButton driveSlowButton = new JoystickButton(driver, leftButtonJoystick); // TODO Rename
    public static final JoystickButton slewRatedDriveCommandButton = new JoystickButton(driver, rightButtonJoystick);
    public static final JoystickButton climbUpButton = new JoystickButton(driver, xButton);
    public static final JoystickButton autoIntakeStartButton = new JoystickButton(driver, aButton);

    public static final JoystickButton indexWrongBallOutButton = new JoystickButton(operator, leftBumper);
    public static final JoystickButton intakeButton = new JoystickButton(operator, rightBumper);
    public static final JoystickButton shootHighAutomaticButton = new JoystickButton(operator, aButton);

    // Joysticks and triggers
    private static final int rightTrigger = 3;
    private static final int leftTrigger = 2;
    private static final int leftStickX = 0;
    public static final int leftStickY = 1; // TODO Figure out how to make this private
    private static final int rightStickX = 4;
    private static final int rightStickY = 5;

    // Center to center distance between left and right modules on the robot (17.5 in. to meters)
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .4445;
    // Center to center distance between front and back modules on the robot (24.5 in. to metes)
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5969;

    // Front left drive ID, steer ID, encoder ID, offset
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(24.6917724609375); //FIXME
    
    // Front right drive ID, steer ID, encoder ID, offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 16;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(301.190185546875); //FIXME

    // Back left drive ID, steer ID, encoder ID, offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 19; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(45.16754150390625); //FIXME
    // Back right drive ID, steer ID, encoder ID, offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 20; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 21; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(77.8656005859375); //FIXME

    //Multipliers
    public static final double driveSlow = .4; //FIXME
    public static final double driveNormal = .8; //FIXME
    private static final double maxVoltage = 12;
    public static final double teleopMaxAccel = 8;
    public static final double teleopMaxAngularAccel = 8;

    //
    // Shooter subsystem info
    //
    private static final int shooterMotor1ID = 30;
    private static final int shooterMotor2ID = 31;
    public static final double shootHigh = .75 * maxVoltage; //73, 78
    public static final double shootLow = .4 * maxVoltage; //FIXME
    public static final double autoShootTime = 3; //FIXME

    public static final CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);

    //
    // Intake subsystem info
    //
    private static final int intakeMotorID = 32;
    public static final double intakeBack = .2; //FIXME
    public static final double intakeForward = -.4; //FIXME
    public static final double autoIntakeTime = 1.85; //FIXME
    public static final int intakeSolenoidIn = 12; // 5
    public static final int intakeSolenoidOut = 13; // 6
    public static final CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    //
    // Indexer subsystem info
    //
    private static final int indexerMotorID = 33;
    private static final int intakeSensorID = 0;
	private static final int ballOneSensorID = 1;
	private static final int ballTwoSensorID = 2;
	public static final double indexSpeedForward = -.3; //FIXME
	public static final double indexSpeedSimple = .3; //FIXME
    public static final double indexSpeedSlow = .1;
    public static final double indexWrongBallOut = -.6;
    public static final double indexSpeedSimpleBack = -.2;
    public static final double readyIndexForShoot = .4;

    public static final CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.intakeSolenoidIn, Constants.intakeSolenoidOut);

    //
    // Climber subsystem info
    //
    private static final int climberLeftInID = 8; //1
    private static final int climberLeftOutID = 9; //2
    private static final int climberRightInID = 10; //3
    private static final int climberRightOutID = 11; //4

    public static final DoubleSolenoid climberLeft = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.climberLeftInID, Constants.climberLeftOutID);
    public static final DoubleSolenoid climberRight = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.climberRightInID, Constants.climberRightOutID);
}
