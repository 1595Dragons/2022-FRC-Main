// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.AutoWaitThenDriveTemplate;
import frc.robot.Util.AutoWaitThenIntakeTemplate;
import frc.robot.Util.AutoWaitThenShootSequence;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBallRight extends SequentialCommandGroup {
  DrivetrainSubsystem m_drivetrainSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;  
  public AutoTwoBallRight(DrivetrainSubsystem m_drivetrainSubsystem, IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_shooterSubsystem) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_shooterSubsystem = m_shooterSubsystem;
    addCommands(
      new AutoWaitThenDriveTemplate(m_drivetrainSubsystem, 1, 0, 0, 0, 1.6).alongWith(
        new AutoWaitThenIntakeTemplate(m_intakeSubsystem, m_indexerSubsystem, 0, 2)),
      new AutoWaitThenDriveTemplate(m_drivetrainSubsystem, -1, 0, 0, .25, 2.9).alongWith(
        new AutoWaitThenShootSequence(m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem, 2))
    );
  }
}
