// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax shooterMotor1, shooterMotor2;
  Compressor compressor;
  public ShooterSubsystem() {
    shooterMotor1 = new CANSparkMax(Constants.shooterMotor1ID, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(Constants.shooterMotor2ID, MotorType.kBrushless);
    compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    
  }
  public void shootLow() {
    shooterMotor1.setVoltage(Constants.shootLow);
    shooterMotor2.setVoltage(-Constants.shootLow);
  }
  
  public void shootHigh() {
    shooterMotor1.setVoltage(Constants.shootHigh);
    shooterMotor2.setVoltage(-Constants.shootHigh);
    compressor.disable();
  }

  public void shootStop() {
    shooterMotor1.setVoltage(0);
    shooterMotor2.setVoltage(0);
    compressor.enabled();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
