package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldRelativeDrive extends SwerveDrive {


    public FieldRelativeDrive(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
                              DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        super(drivetrainSubsystem, translationXSupplier, translationYSupplier, rotationSupplier);
    }

    @Override
    public void execute() {

        ChassisSpeeds driveSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble(), drivetrainSubsystem.getGyroscopeRotation());

        drivetrainSubsystem.drive(driveSpeed);
    }
}
