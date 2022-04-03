// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.autonomous.AutoIntake;
import frc.robot.robotmap.Intake;

public class IntakeSubsystem extends SubsystemBase {

	public AutoIntake automaticIntake;

	public IntakeSubsystem(IndexerSubsystem indexerSubsystem) {

		this.automaticIntake = new AutoIntake(this, indexerSubsystem);
		Intake.intakeSolenoid.set(Value.kReverse);
	}

	public void intakeForward() {
		Intake.intakeMotor.set(Intake.intakeForward);
	}

	public void intakeBackward() {
		Intake.intakeMotor.set(Intake.intakeBack);
	}

	public void intakeStop() {
		Intake.intakeMotor.set(0);
	}

	public void intakeUp() {
		Intake.intakeMotor.set(0);
		Intake.intakeSolenoid.set(Value.kReverse);
	}

	public void intakeDown() {
		Intake.intakeSolenoid.set(Value.kForward);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
