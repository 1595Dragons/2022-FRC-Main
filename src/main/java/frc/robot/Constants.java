// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class OIConstants {
        // Buttons
        public static final int aButton = 1;
        public static final int bButton = 2;
        public static final int yButton = 4;
        public static final int xButton = 3;
        public static final int backButton = 7;
        public static final int startButton = 8;
        public static final int leftBumper = 5;
        public static final int rightBumper = 6;
        public static final int leftButtonJoystick = 9;
        public static final int rightButtonJoystick = 10;

        // Joysticks and triggers
        public static final int rightTrigger = 3;
        public static final int leftTrigger = 2;
        public static final int leftStickX = 0;
        public static final int leftStickY = 1;
        public static final int rightStickX = 4;
        public static final int rightStickY = 5;

    }

    // Center to center distance between left and right modules on the robot (17.5 in. to meters)
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .4445;
    // Center to center distance between front and back modules on the robot (24.5 in. to metes)
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5969;

    // Front left drive ID, steer ID, encoder ID, offset
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(24.609375); //24.9609375
    
    // Front right drive ID, steer ID, encoder ID, offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 16;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(301.11328125); //301.2890625

    // Back left drive ID, steer ID, encoder ID, offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 19; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(45.08514404296875); //44.72808837890625
    // Back right drive ID, steer ID, encoder ID, offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 20; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 21; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(78.75); //78.134765625

    //Multipliers
    public static final double driveSlow = .4; //
    public static final double driveNormal = .5; // 1
    private static final double maxVoltage = 12;
    public static final double teleopMaxAccel = 10.5; // 9.5
    public static final double teleopMaxAngularAccel = 40;
    
    // Shooter subsystem info
    public static final int shooterMotor1ID = 30;
    public static final int shooterMotor2ID = 31;
    public static final double shootHigh = 1 * maxVoltage; //73, 78
    public static final double shootLow = .4 * maxVoltage; //FIXME
    public static final double autoShootTime = 3; //FIXME
    
    // Intake subsystem info
    public static final int intakeMotorID = 32;
    public static final double intakeBack = .2; //FIXME
    public static final double intakeForward = -1; //FIXME
    public static final double autoIntakeTime = 1.85; //FIXME
    public static final int intakeSolenoidIn = 12; // 5
    public static final int intakeSolenoidOut = 13; // 6

    // Indexer subsystem info
    public static final int indexerMotorID = 33;
    public static final int intakeSensorID = 0;
	public static final int ballOneSensorID = 1;
	public static final int ballTwoSensorID = 2;
	public static final double indexSpeedForward = -.3 * maxVoltage; //FIXME
	public static final double indexSpeedSimple = .3 * maxVoltage; //FIXME
    public static final double indexSpeedSlow = .2 * maxVoltage; //.38
    public static final double indexWrongBallOut = -.6 * maxVoltage;
    public static final double indexSpeedSimpleBack = -.2 * maxVoltage;
    public static final double readyIndexForShoot = .4;


    // Climber subsystem info
    public static final int climberLeftInID = 8; //1
    public static final int climberLeftOutID = 9; //2
    public static final int climberRightInID = 10; //3
    public static final int climberRightOutID = 11; //4
    public static final int climberPullInID = 14;
    public static final int climberPullOutID = 15;

    // Limelight Stuff
    public static final class Limelight {
        public static final double MOUNTED_ANGLE = 30; // Degrees
        public static final double LENS_HEIGHT = 36.73; // Inches
        public static final double TARGET_HEIGHT = 103; // Inches

        public static final double LENS_TO_SHOOTER = 11.78; // Inches
    }

    public static final class DriveBaseConstants {
        /**
         * TODO: Fix TRACKWIDTH & WHEELBASE Based On Production Robot
         * (In Meters)
         */

        public static final double TRACKWIDTH = .4445; // Distance From Left Wheel
                                                                            // Middle To Right Wheel Middle
        public static final double WHEELBASE = .5969; // Distance From Front Wheel
                                                                           // Middle To Back Wheel Middle

        public static final double MAX_VOLTAGE = 12.0;

        public static final double X_P_COEFF = 3;
        public static final double Y_P_COEFF = 3; 
        public static final double THETA_P_COEFF = 4;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.5;
        
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final TrajectoryConfig MAX_SPEED_CONFIG = new TrajectoryConfig(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrajectoryConfig MEDIUM_SPEED_CONFIG = new TrajectoryConfig(
                0.6 * MAX_VELOCITY_METERS_PER_SECOND,
                0.6 * MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrajectoryConfig SLOW_SPEED_CONFIG = new TrajectoryConfig(
                0.3 * MAX_VELOCITY_METERS_PER_SECOND,
                0.3 * MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrajectoryConfig AUTO_SPEED_CONFIG = new TrajectoryConfig(
                0.5 * MAX_VELOCITY_METERS_PER_SECOND,
                1.0 * MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double SLOWMODE_MULTIPLIER = 1 / 4;

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                // Front left
                new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                // Front right
                new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                // Back left
                new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                // Back right
                new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));

        public static final double ALIGN_P_COEFF = 0.1;
        public static final double ALIGN_I_COEFF = 0.8;
        public static final double ALIGN_D_COEFF = 0.000;

        public static final double ALIGN_TOLERANCE = .5;
    }
}
