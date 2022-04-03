// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

@Deprecated
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
}
