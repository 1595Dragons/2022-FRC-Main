package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

class SwerveDrive extends CommandBase {

	final DrivetrainSubsystem drivetrainSubsystem;

	final DoubleSupplier translationXSupplier, translationYSupplier, rotationSupplier;

	// Hard coded stop speed for when end is called.
	private final ChassisSpeeds STOP_SPEED = new ChassisSpeeds(0.0, 0.0, 0.0);

	public SwerveDrive(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
	                   DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {

		this.drivetrainSubsystem = drivetrainSubsystem;
		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		addRequirements(drivetrainSubsystem);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrainSubsystem.drive(STOP_SPEED);
	}

}
