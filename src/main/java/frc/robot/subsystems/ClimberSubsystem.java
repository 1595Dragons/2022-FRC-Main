// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

	public void raiseClimber() {
		Constants.climberLeft.set(Value.kForward);
		Constants.climberRight.set(Value.kForward);
	}

	public void lowerClimber() {
		Constants.climberLeft.set(Value.kReverse);
		Constants.climberRight.set(Value.kReverse);
	}

	public void climberOff() {
		Constants.climberLeft.set(Value.kOff);
		Constants.climberRight.set(Value.kOff);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
