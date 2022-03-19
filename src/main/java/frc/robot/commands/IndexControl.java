// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexControl extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  public IndexControl(IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    addRequirements(m_indexerSubsystem);
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerSubsystem.indexBallsControl(RobotContainer.m_operator, Constants.indexSpeedForward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
