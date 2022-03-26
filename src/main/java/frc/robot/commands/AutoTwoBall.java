// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public AutoTwoBall(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem) {
    double 
      maxV = 5, 
      maxA = 3,
      p = .4,
      i = 0,
      d = .025;

    Trajectory autoTwoBallForward = PathPlanner.loadPath("AutoTwoBallPtOne", maxV, maxA, false);
    Trajectory autoTwoBallBack = PathPlanner.loadPath("AutoTwoBallPtTwo", maxV, maxA, false);

    PIDController xController = new PIDController(p , i, d);
    PIDController yController = new PIDController(p, i, d);
    var thetaController = new ProfiledPIDController(5 , 0, 0, new TrapezoidProfile.Constraints(maxV, maxA));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand pt1 = new SwerveControllerCommand(
      autoTwoBallForward, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
      
    SwerveControllerCommand pt2 = new SwerveControllerCommand(
      autoTwoBallBack, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    WaitCommand m_wait = new WaitCommand(5);
    WaitCommand shootRevTime = new WaitCommand(1);
      
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(autoTwoBallForward.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem),
      m_wait,
      new ParallelCommandGroup(pt1, new AutoIntakeStart(m_intakeSubsystem, m_indexerSubsystem)),
      pt2,
      new AutoReadyIndex(m_indexerSubsystem, m_intakeSubsystem, m_shooterSubsystem),
      shootRevTime,
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem)
      );
  }
}
