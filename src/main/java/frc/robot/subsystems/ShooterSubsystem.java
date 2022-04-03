// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

	public void shootLow() {
		Constants.shooterMotor1.set(Constants.shootLow);
		Constants.shooterMotor2.set(-Constants.shootLow);
	}

	public void shootHigh() {
		Constants.shooterMotor1.set(Constants.shootHigh);
		Constants.shooterMotor2.set(-Constants.shootHigh);
	}

	public void shootStop() {
		Constants.shooterMotor1.set(0);
		Constants.shooterMotor2.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
