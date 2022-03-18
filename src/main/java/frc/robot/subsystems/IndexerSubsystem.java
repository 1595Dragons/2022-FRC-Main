// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  
  CANSparkMax indexerMotor;
  public IndexerSubsystem() {
    indexerMotor = new CANSparkMax(Constants.indexerMotorID, MotorType.kBrushless);
  }

  
  public void indexStop() {
    indexerMotor.set(0);
  }

  public void indexBallsControl(XboxController controller, double indexSpeed) {
    indexerMotor.set(controller.getRawAxis(OIConstants.leftStickY) * indexSpeed);
  }

  public void indexBallSimple() {
    indexerMotor.set(Constants.indexSpeedSimple);
  }

  public void indexWrongBallOut() {
    indexerMotor.set(Constants.indexWrongBallOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
