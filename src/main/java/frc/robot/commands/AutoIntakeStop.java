// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeStop extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  Boolean isFinished = false;
  public AutoIntakeStop(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    addRequirements(m_intakeSubsystem);
    addRequirements(m_indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intakeStop();
    m_indexerSubsystem.indexStop();
    m_intakeSubsystem.intakeUp();
    isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
