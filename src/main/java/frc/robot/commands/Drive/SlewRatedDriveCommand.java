package frc.robot.commands.Drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class SlewRatedDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

    public SlewRatedDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        
        xLimiter = new SlewRateLimiter(Constants.teleopMaxAccel);
        yLimiter = new SlewRateLimiter(Constants.teleopMaxAccel);
        thetaLimiter = new SlewRateLimiter(Constants.teleopMaxAngularAccel);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(m_translationXSupplier.getAsDouble());
        double ySpeed = yLimiter.calculate(m_translationYSupplier.getAsDouble());
        double thetaSpeed = thetaLimiter.calculate(m_rotationSupplier.getAsDouble());
        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        thetaSpeed,
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
