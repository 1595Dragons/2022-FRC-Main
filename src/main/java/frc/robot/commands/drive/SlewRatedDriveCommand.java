package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SlewRatedDriveCommand extends SwerveDrive {

    SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public SlewRatedDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                 DoubleSupplier translationXSupplier,
                                 DoubleSupplier translationYSupplier,
                                 DoubleSupplier rotationSupplier) {
        super(drivetrainSubsystem, translationXSupplier, translationYSupplier, rotationSupplier);

        xLimiter = new SlewRateLimiter(DrivetrainSubsystem.teleopMaxAccel);
        yLimiter = new SlewRateLimiter(DrivetrainSubsystem.teleopMaxAccel);
        thetaLimiter = new SlewRateLimiter(DrivetrainSubsystem.teleopMaxAngularAccel);
    }

    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(translationXSupplier.getAsDouble());
        double ySpeed = yLimiter.calculate(translationYSupplier.getAsDouble());
        double thetaSpeed = thetaLimiter.calculate(rotationSupplier.getAsDouble());
        drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
                drivetrainSubsystem.getGyroscopeRotation())
        );
    }
}
