// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Util.SwerveControllerCommandTemplate;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPIDTest extends SequentialCommandGroup {
  /** Creates a new AutoPIDTest. */
  public AutoPIDTest(DrivetrainSubsystem m_drivetrainSubsystem, double p, double i, double d, double thetaP, double maxVel, double maxAccel) {
    addRequirements(m_drivetrainSubsystem);
    SwerveControllerCommandTemplate m_SCCT = new SwerveControllerCommandTemplate();

    Trajectory pidAuto = PathPlanner.loadPath("PIDTestAuto", maxVel, maxAccel);
    
    SwerveControllerCommand pt1 = m_SCCT.SwerveControllerCommandPID(m_drivetrainSubsystem, pidAuto, p, i, d, thetaP, maxVel, maxAccel);

    addCommands(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(pidAuto.getInitialPose())),
      pt1
    );
  }
}
