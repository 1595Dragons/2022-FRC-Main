// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Intake;
import frc.robot.commands.autonomous.AutoRightTwoBall;
import frc.robot.commands.autonomous.AutoSimple;
import frc.robot.robotmap.Controllers;
import frc.robot.robotmap.Indexer;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private Command m_autonomousCommand;

	SendableChooser<Command> autonomousChooser = new SendableChooser<>();

	// Initialize all subsystems in robotInit
	private DrivetrainSubsystem m_drivetrainSubsystem;
	private ShooterSubsystem m_shooterSubsystem;
	private IntakeSubsystem m_intakeSubsystem;
	private IndexerSubsystem m_indexerSubsystem;
	private ClimberSubsystem m_climberSubsystem;

	// Initialize all autonomous options in robotInit
	private AutoSimple simpleAuto;
	private AutoRightTwoBall twoBallAuto;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {

		this.m_drivetrainSubsystem = new DrivetrainSubsystem();
		this.m_shooterSubsystem = new ShooterSubsystem();
		this.m_indexerSubsystem = new IndexerSubsystem(m_shooterSubsystem);
		this.m_intakeSubsystem = new IntakeSubsystem(m_indexerSubsystem);
		this.m_climberSubsystem = new ClimberSubsystem();

		// Autonomous options
		this.simpleAuto = new AutoSimple(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem);
		this.twoBallAuto = new AutoRightTwoBall(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem);

		// SmartDashboard Stuff
		autonomousChooser.setDefaultOption("Simple Auto", simpleAuto);
		autonomousChooser.addOption("Two Ball Auto Right", twoBallAuto);
		SmartDashboard.putData(autonomousChooser);

		// Driver button bindings
		Controllers.resetRobotOrientation.whenPressed(m_drivetrainSubsystem.resetGyro);
		Controllers.driveSlowButton.toggleWhenPressed(m_drivetrainSubsystem.slowerDrive);
		Controllers.climbButton.toggleWhenPressed(m_climberSubsystem.climbUp);
		Controllers.autoIntakeStartButton.whenPressed(m_intakeSubsystem.automaticIntake.withTimeout(2));

		// Operator button bindings
		Controllers.indexWrongBallOutButton.whileHeld(m_indexerSubsystem.wrongBallEject);
		Controllers.intakeButton.whileHeld(new Intake(m_intakeSubsystem, m_indexerSubsystem));
		Controllers.shootHighAutomaticButton.whenPressed(m_indexerSubsystem.readyIndex.withTimeout(Indexer.readyIndexForShoot).andThen(m_shooterSubsystem.readyForHighShot));
		Controllers.shootHighAutomaticButton.whenReleased(m_indexerSubsystem.outputBalls.withTimeout(4));

		//this.startUSBCamera();
	}

	private void startUSBCamera() {
		new Thread(() -> {
			UsbCamera camera = CameraServer.startAutomaticCapture();
			camera.setResolution(640, 480);

			CvSink cvSink = CameraServer.getVideo();
			CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

			Mat source = new Mat();
			Mat output = new Mat();

			while (!Thread.interrupted()) {
				cvSink.grabFrame(source);
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}
		}).start();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your {@link SendableChooser} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = autonomousChooser.getSelected();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		// If you want the autonomous to continue until interrupted by another command,
		// remove this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
