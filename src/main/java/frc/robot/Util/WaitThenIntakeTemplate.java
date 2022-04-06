// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.AutoIntake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitThenIntakeTemplate extends SequentialCommandGroup {
  IntakeSubsystem m_intakeSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  public WaitThenIntakeTemplate(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem, double waitTime, double intakeTime) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem =  m_indexerSubsystem;
    
    addCommands(
      new WaitCommand(waitTime),
      new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(intakeTime)
    );
  }
}
