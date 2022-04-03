// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.SecondaryDriveCommand;
import frc.robot.robotmap.Controllers;

public class DrivetrainSubsystem extends SubsystemBase {

	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			SdsModuleConfigurations.MK4_L2.getDriveReduction() *
			SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

	public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
	);

	private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public DefaultDriveCommand defaultDrive;

	public SecondaryDriveCommand slowerDrive;

	public InstantCommand resetGyro;

	public DrivetrainSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(0, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
				Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
				Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
				Constants.FRONT_LEFT_MODULE_STEER_OFFSET
		);

		m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(2, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
				Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
				Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
				Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
		);

		m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(4, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
				Constants.BACK_LEFT_MODULE_STEER_MOTOR,
				Constants.BACK_LEFT_MODULE_STEER_ENCODER,
				Constants.BACK_LEFT_MODULE_STEER_OFFSET
		);

		m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList)
						.withSize(2, 4)
						.withPosition(6, 0),
				Mk4SwerveModuleHelper.GearRatio.L2,
				Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
				Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
				Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
				Constants.BACK_RIGHT_MODULE_STEER_OFFSET
		);


		this.defaultDrive = new DefaultDriveCommand(this,
				() -> -modifyAxis(Controllers.driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controllers.driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controllers.driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal);

		this.slowerDrive = new SecondaryDriveCommand(this,
				() -> -modifyAxis(Controllers.driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controllers.driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controllers.driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal);

		this.resetGyro = new InstantCommand(m_navx::zeroYaw);

		this.setDefaultCommand(defaultDrive);
	}

	private static double modifyAxis(double value) {

		// Deadband value
		value = deadband(value, 0.1);
		value = Math.copySign(value * value, value);

		return value;
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	public Rotation2d getGyroscopeRotation() {
		if (m_navx.isMagnetometerCalibrated()) {
			return Rotation2d.fromDegrees(m_navx.getFusedHeading());
		}
		return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

	public Pose2d getPose() {
		return odometer.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometer.resetPosition(pose, getGyroscopeRotation());
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[0].angle.getRadians());
		m_frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[1].angle.getRadians());
		m_backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[2].angle.getRadians());
		m_backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, desiredStates[3].angle.getRadians());
	}

	public void stopModules() {
		m_frontLeftModule.set(0, 0);
		m_frontRightModule.set(0, 0);
		m_backLeftModule.set(0, 0);
		m_backRightModule.set(0, 0);
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
	}
}
