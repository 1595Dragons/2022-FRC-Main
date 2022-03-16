// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTestAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public PIDTestAuto(DrivetrainSubsystem m_drivetrainSubsystem) {
    Trajectory PIDTestX = PathPlanner.loadPath("PIDTestX", 3, 3);
    //Trajectory PIDTestY = PathPlanner.loadPath("PIDTestY", 3, 3);
    //Trajectory PIDTestTheta = PathPlanner.loadPath("PIDTestTheta", 3, 3);

    //WaitCommand m_wait = new WaitCommand(SmartDashboard.getNumber("Wait Time", 0));

    PIDController xController = new PIDController(.05, .05, .05);
    PIDController yController = new PIDController(.05, .05, .05);
    ProfiledPIDController thetaController = new ProfiledPIDController(.05, .05, .05, new TrapezoidProfile.Constraints(3, 3));

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
