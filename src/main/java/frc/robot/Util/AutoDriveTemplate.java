// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveTemplate extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  public AutoDriveTemplate(DrivetrainSubsystem m_drivetrainSubsystem, double xPower, double yPower, double turnPower, double driveTime) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
  }

  @Override
  public void execute() {
    new SecondaryDriveCommand(m_drivetrainSubsystem, () -> xPower, () -> yPower, () -> turnPower).withTimeout(driveTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
