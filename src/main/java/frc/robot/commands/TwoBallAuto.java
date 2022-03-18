// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(DrivetrainSubsystem m_drivetrainSubsystem, ShooterSubsystem m_shooterSubsystem, IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem) {
    Trajectory TwoBallAutoPt1 = PathPlanner.loadPath("2BallAutoPt1", 3, 3, true);
    Trajectory TwoBallAutoPt2 = PathPlanner.loadPath("2BallAutoPt2", 1, 1, true);
    Trajectory TwoBallAutoPt3 = PathPlanner.loadPath("2BallAutoPt3", 3, 3, true);
    double MAX_VOLTAGE = 12.0;
    double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
    double xPID = 1.5;
    double yPID = 1.5;
    PIDController xController = new PIDController(xPID , yPID, 0);
    PIDController yController = new PIDController(xPID, yPID, 0);
    var thetaController = new ProfiledPIDController(3 , 0, 0, new TrapezoidProfile.Constraints(MAX_VOLTAGE * MAX_VELOCITY_METERS_PER_SECOND, MAX_VOLTAGE * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
      SwerveControllerCommand pt1 = new SwerveControllerCommand(
      TwoBallAutoPt1, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    SwerveControllerCommand pt2 = new SwerveControllerCommand(
      TwoBallAutoPt2, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
      
    SwerveControllerCommand pt3 = new SwerveControllerCommand(
      TwoBallAutoPt3, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    AutoShootHigh m_autoShootHigh = new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem);
    AutoIntake m_autoIntake = new AutoIntake(m_intakeSubsystem);
    AutoFirstIndex m_autoFirstIndex = new AutoFirstIndex(m_indexerSubsystem, m_intakeSubsystem);
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(TwoBallAutoPt1.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem),
      pt1,
      new ParallelCommandGroup(
        m_autoIntake,
        pt2),
      m_autoFirstIndex,
      pt3,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem)
      );
  }
}
