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
public class ThreeBallShort extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public ThreeBallShort(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem) {
    Trajectory ThreeBallShortPt1 = PathPlanner.loadPath("3BallShortPt1", 3, 3);
    Trajectory ThreeBallShortPt2 = PathPlanner.loadPath("3BallShortPt2", 1, 1.5);
    Trajectory ThreeBallShortPt3 = PathPlanner.loadPath("3BallShortPt3", 3, 3);
    Trajectory ThreeBallShortPt4 = PathPlanner.loadPath("3BallShortPt4", 1, 1.5);
    Trajectory ThreeBallShortPt5 = PathPlanner.loadPath("3BallShortPt5", 5, 3);
    Trajectory ThreeBallShortTaxi = PathPlanner.loadPath("3BallShortTaxi", 5, 3);

    PIDController xController = new PIDController(.05, .05, .05);
    PIDController yController = new PIDController(.05, .05, .05);
    ProfiledPIDController thetaController = new ProfiledPIDController(.05, .05, .05, new TrapezoidProfile.Constraints(3, 3));

    SwerveControllerCommand pt1 = new SwerveControllerCommand(
      ThreeBallShortPt1, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    SwerveControllerCommand pt2 = new SwerveControllerCommand(
      ThreeBallShortPt2, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
      
    SwerveControllerCommand pt3 = new SwerveControllerCommand(
      ThreeBallShortPt3, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt4 = new SwerveControllerCommand(
      ThreeBallShortPt4, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand pt5 = new SwerveControllerCommand(
      ThreeBallShortPt5, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    SwerveControllerCommand taxi = new SwerveControllerCommand(
      ThreeBallShortTaxi, 
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
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(ThreeBallShortPt1.getInitialPose())),
      m_autoShootHigh,
      pt1,
      new ParallelCommandGroup(pt2, m_autoIntake),
      pt3,
      new ParallelCommandGroup(pt4, m_autoIntake),
      pt5,
      m_autoShootHigh,
      taxi
    );
  }
}
