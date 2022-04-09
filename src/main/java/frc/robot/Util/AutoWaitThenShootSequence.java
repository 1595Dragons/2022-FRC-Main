// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.AutoReadyIndexToShoot;
import frc.robot.commands.Autonomous.AutoShootHigh;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoWaitThenShootSequence extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  double waitTime;
  public AutoWaitThenShootSequence(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem, ShooterSubsystem m_shooterSubsystem, double waitTime) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_shooterSubsystem = m_shooterSubsystem;
    this.waitTime = waitTime;

    addCommands(
      new WaitCommand(waitTime),
      new AutoReadyIndexToShoot(m_indexerSubsystem, m_shooterSubsystem).withTimeout(.2),
      new WaitCommand(1.65),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(1.45)
    );
  }
}
