// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutoIntake;
import frc.robot.commands.autonomous.AutoRightTwoBall;
import frc.robot.commands.autonomous.AutoSimple;
import frc.robot.robotmap.Controls;
import frc.robot.robotmap.Indexer;
import frc.robot.subsystems.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private Command m_autonomousCommand;

	SendableChooser<Command> autonomousChooser = new SendableChooser<>();

	private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
	private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {

		// TODO MOve to driver subsystem.
		m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
				m_drivetrainSubsystem,
				() -> -modifyAxis(Controls.driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controls.driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controls.driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

		m_indexerSubsystem.setDefaultCommand(new IndexControl(m_indexerSubsystem));
		m_climberSubsystem.setDefaultCommand(new ClimbDown(m_climberSubsystem));

		//SmartDashboard Stuff
		autonomousChooser.setDefaultOption("Simple Auto", new AutoSimple(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem));
		autonomousChooser.addOption("Two Ball Auto Right", new AutoRightTwoBall(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
		SmartDashboard.putData(autonomousChooser);

		// Driver button bindings
		Controls.resetRobotOrientation.whenPressed(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

		Controls.driveSlowButton.toggleWhenPressed(new SecondaryDriveCommand(
				m_drivetrainSubsystem,
				() -> -modifyAxis(Controls.driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controls.driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
				() -> -modifyAxis(Controls.driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

		Controls.climbButton.toggleWhenPressed(new ClimbUp(m_climberSubsystem));
		Controls.autoIntakeStartButton.whenPressed(new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(2));


		// Operator button bindings
		Controls.indexWrongBallOutButton.whileHeld(new IndexWrongBallOut(m_indexerSubsystem));
		Controls.intakeButton.whileHeld(new Intake(m_intakeSubsystem, m_indexerSubsystem));

		Controls.shootHighAutomaticButton.whenPressed(new ReadyIndex(m_indexerSubsystem, m_shooterSubsystem)
				.withTimeout(Indexer.readyIndexForShoot)
				.andThen(new ReadyShooterHigh(m_indexerSubsystem, m_shooterSubsystem)));
		Controls.shootHighAutomaticButton.whenReleased(new OutputBallsToShoot(m_shooterSubsystem, m_indexerSubsystem).withTimeout(4));

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

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value) {

		// Deadband value
		value = deadband(value, 0.1);
		value = Math.copySign(value * value, value);

		return value;
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
