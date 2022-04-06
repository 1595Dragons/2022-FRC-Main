// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.commands.Drive.SecondaryDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBallSimpleOld extends SequentialCommandGroup {
  DrivetrainSubsystem m_drivetrainSubsystem;
  IndexerSubsystem m_indexerSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  double 
    powerX = 1,
    powerY = 0,
    powerTheta2 = 0.02,
    powerTheta = 0;
  
  public AutoTwoBallSimpleOld(DrivetrainSubsystem m_drivetrainSubsystem, IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem, ShooterSubsystem m_shooterSubsystem) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_shooterSubsystem = m_shooterSubsystem;
    addCommands(
      new AutoReadyIndexToShoot(m_indexerSubsystem, m_shooterSubsystem).withTimeout(.25),
      new WaitCommand(1.5),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(.5),
      new WaitCommand(1),
      new SecondaryDriveCommand(m_drivetrainSubsystem, () -> powerX, () -> powerY,  () -> powerTheta).withTimeout(2.75).alongWith(new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(4)),
      new WaitCommand(.5),
      new SecondaryDriveCommand(m_drivetrainSubsystem, () -> -powerX, () -> powerY,  () -> powerTheta2).withTimeout(2.9),
      new AutoReadyIndexToShoot(m_indexerSubsystem, m_shooterSubsystem).withTimeout(.25),
      new WaitCommand(1.5),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(.5)

      );
  }
}
