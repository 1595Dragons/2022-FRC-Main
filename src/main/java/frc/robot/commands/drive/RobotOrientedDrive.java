// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RobotOrientedDrive extends SwerveDrive {

	private final ChassisSpeeds robotOrientated;

	public RobotOrientedDrive(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
	                          DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
		super(drivetrainSubsystem);

		// You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
		this.robotOrientated = new ChassisSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		drivetrainSubsystem.drive(robotOrientated);
	}
}