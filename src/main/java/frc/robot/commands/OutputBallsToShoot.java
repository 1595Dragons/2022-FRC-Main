// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OutputBallsToShoot extends CommandBase {
  ShooterSubsystem m_shooterSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  public OutputBallsToShoot(ShooterSubsystem m_shooterSubsystem, IndexerSubsystem m_indexerSubsystem) {
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_shooterSubsystem = m_shooterSubsystem;
    addRequirements(m_indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerSubsystem.indexBallSlow();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.indexStop();
    m_shooterSubsystem.shootStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
