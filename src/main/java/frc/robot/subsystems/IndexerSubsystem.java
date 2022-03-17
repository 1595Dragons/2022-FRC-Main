// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  
  CANSparkMax indexerMotor;
  public IndexerSubsystem() {
    indexerMotor = new CANSparkMax(Constants.indexerMotorID, MotorType.kBrushless);
  }

  
  public void stopIndexerMotor() {
    indexerMotor.set(0);
  }

  public void indexBallsForward() {
    indexerMotor.set(Constants.indexSpeedForward);
  }

  public void indexBallsBack() {
    indexerMotor.set(Constants.indexSpeedBack);
  }

  public void IndexWrongBallOut() {
    indexerMotor.set(Constants.indexWrongBallOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
