// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntake extends CommandBase {

  IntakeSubsystem m_intakeSubsystem;
  Timer time;
  Boolean isFinished = false;
  public AutoIntake(IntakeSubsystem m_intakeSubsystem) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    time = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    while (time.get() < Constants.autoIntakeTime) {
      m_intakeSubsystem.intakeDown();
      m_intakeSubsystem.intakeForward();
    }
    isFinished = true;
    m_intakeSubsystem.intakeStop();
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
