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
import frc.robot.Constants;
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
      p = .62,
      i = 0,
      d = .035;

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
    
    WaitCommand m_wait = new WaitCommand(3);
    WaitCommand shootRevTime = new WaitCommand(1);
      
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(autoTwoBallForward.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(2),
      m_wait,
      //pt1,
      //pt2,
      new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(2)
      //new AutoReadyIndex(m_indexerSubsystem, m_shooterSubsystem).withTimeout(Constants.readyIndexForShoot),
      //shootRevTime
      ///new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem).withTimeout(Constants.autoShootTime)
      );
  }
}
