// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax shooterMotor;

  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(Constants.shooterMotorID, MotorType.kBrushless);
  }

  public void shootLow(double shootSpeed) {
    shooterMotor.set(shootSpeed);
  }
  
  public void shootHigh(double shootSpeed) {
    shooterMotor.set(shootSpeed);
  }

  public void shootStop() {
    shooterMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
