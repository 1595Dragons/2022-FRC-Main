// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Center to center distance between left and right modules on the robot (17.5 in. to meters)
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .4445;
    // Center to center distance between front and back modules on the robot (24.5 in. to metes)
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5969;

    // Front left drive ID, steer ID, encoder ID, offset
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(203.810);
    
    // Front right drive ID, steer ID, encoder ID, offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 16;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(121.371);

    // Back left drive ID, steer ID, encoder ID, offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 19; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(167.075); 
    // Back right drive ID, steer ID, encoder ID, offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 20; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 21; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(258.563);
}
