// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoLong extends SequentialCommandGroup {
  /** Creates a new IThreeBallAuto. */
  public ThreeBallAutoLong(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem) {
    Trajectory ThreeBallAutoLongPt1 = PathPlanner.loadPath("3BallAutoLongPt1", 3, 3);
    Trajectory ThreeBallAutoLongPt2 = PathPlanner.loadPath("3BallAutoLongPt2", 1, 1);
    Trajectory ThreeBallAutoLongPt3 = PathPlanner.loadPath("3BallAutoLongPt3", 3, 3);
    Trajectory ThreeBallAutoLongPt4 = PathPlanner.loadPath("3BallAutoLongPt4", 1, 1);
    Trajectory ThreeBallAutoLongPt5 = PathPlanner.loadPath("3BallAutoLongPt5", 3, 3);
    Trajectory ThreeBallAutoLongPt6 = PathPlanner.loadPath("3BallAutoLongPt6", 3, 3);
    
    PIDController xController = new PIDController(.05, .05, .05);
    PIDController yController = new PIDController(.05, .05, .05);
    ProfiledPIDController thetaController = new ProfiledPIDController(.05, .05, .05, new TrapezoidProfile.Constraints(3, 3));

    SwerveControllerCommand pt1 = new SwerveControllerCommand(
      ThreeBallAutoLongPt1, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt2 = new SwerveControllerCommand(
      ThreeBallAutoLongPt2, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt3 = new SwerveControllerCommand(
      ThreeBallAutoLongPt3, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt4 = new SwerveControllerCommand(
      ThreeBallAutoLongPt4, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
      
    SwerveControllerCommand pt5 = new SwerveControllerCommand(
      ThreeBallAutoLongPt5, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt6 = new SwerveControllerCommand(
      ThreeBallAutoLongPt6, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
  
      

    AutoShootHigh m_autoShootHigh = new AutoShootHigh(m_shooterSubsystem, m_intakeSubsystem);
    AutoIntake m_autoIntake = new AutoIntake(m_intakeSubsystem);
    
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(ThreeBallAutoLongPt1.getInitialPose())),
      m_autoShootHigh,
      pt1,
      new ParallelCommandGroup(pt2, m_autoIntake),
      pt3,
      new ParallelCommandGroup(pt4, m_autoIntake),
      pt5,
      m_autoShootHigh,
      pt6,
      new ParallelCommandGroup(
        new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
        new InstantCommand(() -> m_shooterSubsystem.shootStop()),
        new InstantCommand(() -> m_intakeSubsystem.intakeUp()))
    );
  }
}
