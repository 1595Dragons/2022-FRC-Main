// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.SecondaryDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoWaitThenDriveTemplate extends SequentialCommandGroup {
  DrivetrainSubsystem m_drivetrainSubsystem;
  double xPower, yPower, turnPower, driveTime, waitTime;
  public AutoWaitThenDriveTemplate(DrivetrainSubsystem m_drivetrainSubsystem, double xPower, double yPower, double turnPower, double waitTime, double driveTime) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.xPower = xPower;
    this.yPower = yPower;
    this.turnPower = turnPower;
    this.driveTime = driveTime;
    this.waitTime = waitTime;

    addCommands(
      new WaitCommand(waitTime),
      new SecondaryDriveCommand(m_drivetrainSubsystem, () -> xPower, () -> yPower, () -> turnPower).withTimeout(driveTime)
    );
  }

}
