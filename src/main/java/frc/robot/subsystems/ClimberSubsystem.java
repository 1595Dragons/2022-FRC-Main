// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.robotmap.Climber;

public class ClimberSubsystem extends SubsystemBase {

	public ClimbDown climbDown;

	public ClimbUp climbUp;

	public ClimberSubsystem() {

		this.climbDown = new ClimbDown(this);
		this.climbUp = new ClimbUp(this);

		this.setDefaultCommand(climbDown);
	}

	public void raiseClimber() {
		Climber.climberLeft.set(Value.kForward);
		Climber.climberRight.set(Value.kForward);
	}

	public void lowerClimber() {
		Climber.climberLeft.set(Value.kReverse);
		Climber.climberRight.set(Value.kReverse);
	}

	public void climberOff() {
		Climber.climberLeft.set(Value.kOff);
		Climber.climberRight.set(Value.kOff);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
