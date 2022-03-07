// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  
  CANSparkMax indexerMotor;
  DigitalInput intakeSensor, ballOneSensor, ballTwoSensor;
  public IndexerSubsystem() {
    indexerMotor = new CANSparkMax(Constants.indexerMotorID, MotorType.kBrushless);
    intakeSensor = new DigitalInput(Constants.intakeSensorID);
    ballOneSensor = new DigitalInput(Constants.ballOneSensorID);
    ballTwoSensor = new DigitalInput(Constants.ballTwoSensorID);
  }

  public void stopIndexerMotor() {
    indexerMotor.set(0);
  }
  public void indexBallsForward(double indexSpeed) {
    indexerMotor.set(indexSpeed);
  }

  public void indexBallsBack(double indexSpeed) {
    indexerMotor.set(indexSpeed);
  }

  public boolean pullInBall() {
    boolean exitNow = false;
    while (exitNow == false) {
      // Create off conditions
      if (ballOneSensor.get() && ballTwoSensor.get()) {
        stopIndexerMotor();
        exitNow = true;
      }
      else if (!intakeSensor.get()  && !ballOneSensor.get() && !ballTwoSensor.get()) {
        stopIndexerMotor();
      }
      else if (!intakeSensor.get() && !ballOneSensor.get() && ballTwoSensor.get()) {
        indexBallsBack(Constants.indexSpeedBack);
      }
      else if (!intakeSensor.get() && ballOneSensor.get() && !ballTwoSensor.get()) {
        stopIndexerMotor();
      }
      else if (intakeSensor.get() && !ballOneSensor.get() && !ballTwoSensor.get()) {
        indexBallsForward(Constants.indexSpeedForward);
      }
      else if (intakeSensor.get() && ballOneSensor.get() && !ballTwoSensor.get()) {
        indexBallsForward(Constants.indexSpeedForward);
      }
    }
    
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
