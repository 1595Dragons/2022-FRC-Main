// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIndex extends CommandBase {

  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  Timer time = new Timer();
  Boolean isFinished = false;
  public AutoIndex(IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem) {
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    while (time.get() < Constants.autoIndexTime){
      m_indexerSubsystem.indexBallSimple();
    }
    isFinished = true;
    m_indexerSubsystem.indexStop();
    m_intakeSubsystem.intakeUp();
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