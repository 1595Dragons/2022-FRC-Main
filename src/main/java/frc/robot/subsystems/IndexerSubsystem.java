// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  public void indexStop() {
    Constants.indexerMotor.set(0);
  }

  public void indexBallsControl(XboxController controller, double indexSpeed) {
    Constants.indexerMotor.set(controller.getRawAxis(Constants.leftStickY) * indexSpeed);
  }

  public void indexBallSlow() {
    Constants.indexerMotor.set(Constants.indexSpeedSlow);
  }

  public void indexBallSimple() {
    Constants.indexerMotor.set(Constants.indexSpeedSimple);
  }

  public void indexBallSimpleBack() {
    Constants.indexerMotor.set(Constants.indexSpeedSimpleBack);
  }

  public void indexWrongBallOut() {
    Constants.indexerMotor.set(Constants.indexWrongBallOut);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
