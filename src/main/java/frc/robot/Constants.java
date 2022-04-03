// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

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

	// Driveing Multipliers
	public static final double driveSlow = .4; //FIXME
	public static final double driveNormal = .8; //FIXME

	//
	// Shooter subsystem info
	//

	// Shooter motor 1
	private static final int shooterMotor1ID = 30;
	public static final CANSparkMax shooterMotor1 = new CANSparkMax(shooterMotor1ID, CANSparkMaxLowLevel.MotorType.kBrushed);

	// Shooter motor 2
	private static final int shooterMotor2ID = 31;
	public static final CANSparkMax shooterMotor2 = new CANSparkMax(shooterMotor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);

	// Constants for shooter speed.
	public static final double shootHigh = 0.75d; //73, 78
	public static final double shootLow = 0.4d; //FIXME
	public static final double autoShootTime = 3.0d; //FIXME

	//
	// Intake subsystem info
	//

	// Intake motor
	private static final int intakeMotorID = 32;
	public static final CANSparkMax intakeMotor = new CANSparkMax(intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

	public static final double intakeBack = 0.2d; //FIXME
	public static final double intakeForward = -0.4d; //FIXME
	public static final double autoIntakeTime = 1.85d; //FIXME

	private static final int intakeSolenoidIn = 12; // 5
	private static final int intakeSolenoidOut = 13; // 6
	public static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, intakeSolenoidIn, intakeSolenoidOut);

	//
	// Indexer subsystem info
	//

	// Indexer motor
	private static final int indexerMotorID = 33;
	public static final CANSparkMax indexerMotor = new CANSparkMax(indexerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

	//private static final int intakeSensorID = 0;
	//private static final int ballOneSensorID = 1;
	//private static final int ballTwoSensorID = 2;
	public static final double indexSpeedForward = -0.3d; //FIXME
	public static final double indexSpeedSimple = 0.3d; //FIXME
	public static final double indexSpeedSlow = 0.1d;
	public static final double indexWrongBallOut = -0.6d;
	public static final double indexSpeedSimpleBack = -0.2d;
	public static final double readyIndexForShoot = 0.4d;

	//
	// Climber subsystem info
	//

	// Left climber
	private static final int climberLeftInID = 8; //1
	private static final int climberLeftOutID = 9; //2
	public static final DoubleSolenoid climberLeft = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, climberLeftInID, climberLeftOutID);

	// Right climber
	private static final int climberRightInID = 10; //3
	private static final int climberRightOutID = 11; //4
	public static final DoubleSolenoid climberRight = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, climberRightInID, climberRightOutID);
}
