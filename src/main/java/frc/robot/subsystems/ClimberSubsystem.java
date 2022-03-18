// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  
  DoubleSolenoid climberLeft, climberRight;


  public ClimberSubsystem() {
    climberLeft = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.climberLeftInID, Constants.climberLeftOutID);
    climberRight = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.climberRightInID, Constants.climberRightOutID);
  }

  public void raiseClimber() {
    climberLeft.set(Value.kForward);
    climberRight.set(Value.kForward);

  }

  public void lowerClimber() {
    climberLeft.set(Value.kReverse);
    climberRight.set(Value.kReverse);

  }

  public void climberOff() {
    climberLeft.set(Value.kOff);
    climberRight.set(Value.kOff);
  }

  @Override
  public void periodic() {
  }
}
