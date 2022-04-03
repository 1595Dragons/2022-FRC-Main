// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IndexControl;
import frc.robot.commands.IndexWrongBallOut;
import frc.robot.commands.OutputBallsToShoot;
import frc.robot.commands.ReadyIndex;
import frc.robot.robotmap.Controllers;
import frc.robot.robotmap.Indexer;

public class IndexerSubsystem extends SubsystemBase {

	public IndexControl indexControl;
	public IndexWrongBallOut wrongBallEject;
	public ReadyIndex readyIndex;
	public OutputBallsToShoot outputBalls;

	public IndexerSubsystem(ShooterSubsystem shooterSubsystem) {

		this.indexControl = new IndexControl(this);
		this.wrongBallEject = new IndexWrongBallOut(this);
		this.readyIndex = new ReadyIndex(this, shooterSubsystem);
		this.outputBalls = new OutputBallsToShoot(shooterSubsystem, this);

		this.setDefaultCommand(indexControl);
	}

	public void indexStop() {
		Indexer.indexerMotor.set(0);
	}

	public void indexBallsControl(XboxController controller, double indexSpeed) {
		Indexer.indexerMotor.set(controller.getRawAxis(Controllers.leftStickYIndex) * indexSpeed);
	}

	public void indexBallSlow() {
		Indexer.indexerMotor.set(Indexer.indexSpeedSlow);
	}

	public void indexBallSimple() {
		Indexer.indexerMotor.set(Indexer.indexSpeedSimple);
	}

	public void indexBallSimpleBack() {
		Indexer.indexerMotor.set(Indexer.indexSpeedSimpleBack);
	}

	public void indexWrongBallOut() {
		Indexer.indexerMotor.set(Indexer.indexWrongBallOut);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
