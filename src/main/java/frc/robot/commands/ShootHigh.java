// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootHigh extends CommandBase {

  ShooterSubsystem m_shooterSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  public ShootHigh(ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem) {
    this.m_shooterSubsystem = m_shooterSubsystem;
    this.m_intakeSubsystem =  m_intakeSubsystem;
    addRequirements(m_shooterSubsystem);
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.shootHigh(Constants.shootHigh);
    m_intakeSubsystem.ejectBallsToShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.shootStop();
    m_intakeSubsystem.stopIndexerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
