// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.robotmap.Shooter;

public class ShooterSubsystem extends SubsystemBase {

	public void shootLow() {
		Shooter.shooterMotor1.set(Shooter.shootLow);
		Shooter.shooterMotor2.set(-Shooter.shootLow);
	}

	public void shootHigh() {
		Shooter.shooterMotor1.set(Shooter.shootHigh);
		Shooter.shooterMotor2.set(-Shooter.shootHigh);
	}

	public void shootStop() {
		Shooter.shooterMotor1.set(0);
		Shooter.shooterMotor2.set(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
