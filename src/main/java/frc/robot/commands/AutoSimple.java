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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
    
;
    double 
      maxV = 5, 
      maxA = 3,
      p = .4,
      i = 0,
      d = .025;

    Trajectory simpleAuto = PathPlanner.loadPath("PIDTestX", maxV, maxA, true);

    PIDController xController = new PIDController(p , i, d);
    PIDController yController = new PIDController(p, i, d);
    var thetaController = new ProfiledPIDController(5 , 0, 0, new TrapezoidProfile.Constraints(maxV, maxA));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand pt1 = new SwerveControllerCommand(
      simpleAuto, 
      m_drivetrainSubsystem::getPose, 
      m_drivetrainSubsystem.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);
      
    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(simpleAuto.getInitialPose())),
      new AutoShootHigh(m_indexerSubsystem, m_shooterSubsystem),
      pt1
      );
  }
}
