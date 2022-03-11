// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new IThreeBallAuto. */
  public IThreeBallAuto(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shootersubsystem, IntakeSubsystem m_intakesubsystem) {
    Trajectory IThreeBallAutoPt1 = PathPlanner.loadPath("IThreeBallAutoPt1", 3, 3);
    Trajectory IThreeBallAutoPt2 = PathPlanner.loadPath("IThreeBallAutoPt2", 1, 1);
    Trajectory IThreeBallAutoPt3 = PathPlanner.loadPath("IThreeBallAutoPt3", 3, 3);
    Trajectory IThreeBallAutoPt4 = PathPlanner.loadPath("IThreeBallAutoPt4", 1, 1);
    Trajectory IThreeBallAutoPt5 = PathPlanner.loadPath("IThreeBallAutoPt5", 3, 3);
    
    PIDController xController = new PIDController(.05, .05, .05);
    PIDController yController = new PIDController(.05, .05, .05);
    ProfiledPIDController thetaController = new ProfiledPIDController(.05, .05, .05, new TrapezoidProfile.Constraints(3, 3));

    SwerveControllerCommand pt1 = new SwerveControllerCommand(
      IThreeBallAutoPt1, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      SwerveControllerCommand pt2 = new SwerveControllerCommand(
      IThreeBallAutoPt2, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      SwerveControllerCommand pt3 = new SwerveControllerCommand(
      IThreeBallAutoPt3, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      SwerveControllerCommand pt4 = new SwerveControllerCommand(
      IThreeBallAutoPt4, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      SwerveControllerCommand pt5 = new SwerveControllerCommand(
      IThreeBallAutoPt5, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      AutoShootHigh m_autoShootHigh = new AutoShootHigh(m_shootersubsystem);
      AutoIntake m_autoIntake = new AutoIntake(m_intakesubsystem);
    
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(IThreeBallAutoPt1.getInitialPose())),
      m_autoShootHigh,
      pt1,
      new ParallelCommandGroup(pt2, m_autoIntake),
      pt3,
      new ParallelCommandGroup(pt4, m_autoIntake),
      pt5,
      m_autoShootHigh
    );
  }
}
