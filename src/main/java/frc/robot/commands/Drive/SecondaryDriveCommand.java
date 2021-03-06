// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SecondaryDriveCommand extends CommandBase {

  DrivetrainSubsystem m_drivetrainSubsystem;

  DoubleSupplier m_translationXSupplier;
  DoubleSupplier m_translationYSupplier;
  DoubleSupplier m_rotationSupplier;

  public SecondaryDriveCommand(DrivetrainSubsystem m_drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

addRequirements(m_drivetrainSubsystem);
}

@Override
public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    m_drivetrainSubsystem.drive(
            new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble())
    );
}

@Override
public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
}
}