// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.SwerveControllerCommandTemplate;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSimple extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public AutoSimple(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem) {
    addRequirements(m_drivetrainSubsystem);
    addRequirements(m_shooterSubsystem);
    addRequirements(m_intakeSubsystem);
    addRequirements(m_indexerSubsystem);
    SwerveControllerCommandTemplate m_SCCT = new SwerveControllerCommandTemplate();
    double 
      maxVel = 5, 
      maxAccel = 3;

    Trajectory simpleAuto = PathPlanner.loadPath("SimpleAuto", maxVel, maxAccel, false);

    SwerveControllerCommand pt1 = m_SCCT.SwerveControllerCommand(m_drivetrainSubsystem, simpleAuto);
    
    WaitCommand m_wait = new WaitCommand(3);
      
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(simpleAuto.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(2),
      m_wait,
      pt1
      );
  }
}
