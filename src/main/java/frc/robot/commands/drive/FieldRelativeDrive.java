package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class FieldRelativeDrive extends SwerveDrive {

    private final ChassisSpeeds fieldRelative;


    public FieldRelativeDrive(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier translationXSupplier,
                              DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        super(drivetrainSubsystem);

        this.fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
                translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble(), drivetrainSubsystem.getGyroscopeRotation());
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(fieldRelative);
    }
}
