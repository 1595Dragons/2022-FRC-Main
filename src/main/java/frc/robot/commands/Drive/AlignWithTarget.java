// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.Util.HoodPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AlignWithTarget extends CommandBase {
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private final HoodPIDController alignController;
  private double newPower;

  public AlignWithTarget(DrivetrainSubsystem m_drivetrainSubsystem, LimeLightSubsystem m_limeLightSubsystem) {
    this.m_drivetrainSubsystem = m_drivetrainSubsystem;
    this.m_limeLightSubsystem = m_limeLightSubsystem;
    alignController = new HoodPIDController(.1, .8, 0);
    alignController.setTolerance(.5);
    addRequirements(m_drivetrainSubsystem, m_limeLightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newPower = alignController.calculate(m_limeLightSubsystem.getTargetHorizontalOffset());
    if (newPower > 5.75)
      newPower = 5.75;
    if (newPower < -5.75)
      newPower = -5.75;
    m_drivetrainSubsystem.drive(0,0,newPower*1.2);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return alignController.atSetpoint() || !m_limeLightSubsystem.hasValidTarget();
  }
}
