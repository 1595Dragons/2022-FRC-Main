// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.SwerveControllerCommandTemplate;
import frc.robot.robotmap.Indexer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBall extends SequentialCommandGroup {
  public AutoTwoBall(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IndexerSubsystem m_indexerSubsystem, IntakeSubsystem m_intakeSubsystem) {
    addRequirements(m_drivetrainSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_indexerSubsystem);
    addRequirements(m_intakeSubsystem);

    SwerveControllerCommandTemplate m_SCCT = new SwerveControllerCommandTemplate();
    double 
      maxVel = 5, 
      maxAccel = 3;

    Trajectory autoPt1 = PathPlanner.loadPath("AutoTwoBallPtOne", maxVel, maxAccel);
    Trajectory autoPt2 = PathPlanner.loadPath("AutoTwoBallPtTwo", maxVel, maxAccel, true);

    WaitCommand m_wait1 = new WaitCommand(1);

    SwerveControllerCommand pt1 = m_SCCT.SwerveControllerCommand(m_drivetrainSubsystem, autoPt1);
    SwerveControllerCommand pt2 = m_SCCT.SwerveControllerCommand(m_drivetrainSubsystem, autoPt2);

    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(autoPt1.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(2),
      new WaitCommand(3),
      pt1.alongWith(new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(3)),
      m_wait1,
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(autoPt2.getInitialPose())),
      pt2.alongWith(new AutoReadyIndex(m_indexerSubsystem, m_shooterSubsystem).withTimeout(Indexer.readyIndexForShoot)),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(2)
    );
  }
}
