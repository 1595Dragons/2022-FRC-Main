package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.PathPlanner;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTestAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public PIDTestAuto(DrivetrainSubsystem m_drivetrainSubsystem) {
    Trajectory PIDTestX = PathPlanner.loadPath("PIDTestX", 3, 3, true);
    //Trajectory PIDTestY = PathPlanner.loadPath("PIDTestY", 3, 3);
    //Trajectory PIDTestTheta = PathPlanner.loadPath("PIDTestTheta", 3, 3);

    //WaitCommand m_wait = new WaitCommand(SmartDashboard.getNumber("Wait Time", 0));

    double MAX_VOLTAGE = 12.0;
    double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
    double xPID = .4;
    double yPID = 0;
    PIDController xController = new PIDController(xPID , yPID, 0);
    PIDController yController = new PIDController(xPID, yPID, 0);
    var thetaController = new ProfiledPIDController(.05, 0, 0, new TrapezoidProfile.Constraints(MAX_VOLTAGE * MAX_VELOCITY_METERS_PER_SECOND, MAX_VOLTAGE * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand xPt = new SwerveControllerCommand(
      PIDTestX, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
      /*
    SwerveControllerCommand yPt = new SwerveControllerCommand(
      PIDTestY, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
    
    SwerveControllerCommand thetaPt = new SwerveControllerCommand(
      PIDTestTheta, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

      */
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(PIDTestX.getInitialPose())),
      //m_wait,
      xPt,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules())
    );

    /*
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(PIDTestY.getInitialPose())),
      YPt,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules())
    );
    */

    /*
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(PIDTestTheta.getInitialPose())),
      thetaPt,
      new InstantCommand(() -> m_drivetrainSubsystem.stopModules())
    );
    */
  }
}
