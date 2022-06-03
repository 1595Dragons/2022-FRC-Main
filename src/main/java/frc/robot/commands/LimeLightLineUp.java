// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AlignWithTarget;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeLightLineUp extends SequentialCommandGroup {
  
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final LimeLightSubsystem m_limelightSubsystem;
  public LimeLightLineUp(DrivetrainSubsystem m_drivetrainSubsystem, LimeLightSubsystem m_limelightSubsystem) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.m_limelightSubsystem = m_limelightSubsystem;

    addRequirements(m_drivetrainSubsystem, m_limelightSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignWithTarget(m_drivetrainSubsystem, this.m_limelightSubsystem).withTimeout(2)
    );
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_drivetrainSubsystem.drive(0,0,0);
  }
}
