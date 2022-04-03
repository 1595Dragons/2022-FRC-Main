// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ReadyShooterHigh;
import frc.robot.robotmap.Shooter;

public class ShooterSubsystem extends SubsystemBase {

	public ReadyShooterHigh readyForHighShot;

<<<<<<< HEAD
  public void shootLow() {
    shooterMotor1.setVoltage(Constants.shootLow);
    shooterMotor2.setVoltage(-Constants.shootLow);
  }
  
  public void shootHigh() {
    shooterMotor1.setVoltage(Constants.shootHigh);
    shooterMotor2.setVoltage(-Constants.shootHigh);
  }

  public void shootStop() {
    shooterMotor1.setVoltage(0);
    shooterMotor2.setVoltage(0);
  }
=======
	public ShooterSubsystem() {
		this.readyForHighShot = new ReadyShooterHigh(this);
	}

	public void shootLow() {
		Shooter.shooterMotor1.setVoltage(Shooter.shootLow);
		Shooter.shooterMotor2.setVoltage(-Shooter.shootLow);
	}
>>>>>>> main

	public void shootHigh() {
		Shooter.shooterMotor1.setVoltage(Shooter.shootHigh);
		Shooter.shooterMotor2.setVoltage(-Shooter.shootHigh);
	}

	public void shootStop() {
		Shooter.shooterMotor1.setVoltage(0);
		Shooter.shooterMotor2.setVoltage(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
