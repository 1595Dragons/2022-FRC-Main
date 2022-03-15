// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  
  DoubleSolenoid climberLeft, climberRight;
  ClimberState climberState;

  private enum ClimberState {
    Up,
    Down,
    Off
  }
  public ClimberSubsystem() {
    climberLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.climberLeftInID, Constants.climberLeftOutID);
    climberRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.climberRightInID, Constants.climberRightOutID);
  }

  public void raiseClimber() {
    climberLeft.set(Value.kForward);
    climberRight.set(Value.kForward);
    climberState = ClimberState.Up;
  }

  public void lowerClimber() {
    climberLeft.set(Value.kReverse);
    climberRight.set(Value.kReverse);
    climberState = ClimberState.Down;
  }

  public void climberOff() {
    climberLeft.set(Value.kOff);
    climberRight.set(Value.kOff);
    climberState = ClimberState.Off;
  }

  public Boolean currentClimberState() {
    if (climberState == ClimberState.Up) {
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Current Climber State", currentClimberState());
  }
}
