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
import frc.robot.commands.autonomous.AutoPIDTest;
import frc.robot.commands.autonomous.AutoSimple;
import frc.robot.commands.autonomous.AutoTwoBall;
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
	private DrivetrainSubsystem drivetrainSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private IndexerSubsystem indexerSubsystem;
	private ClimberSubsystem climberSubsystem;

	// Initialize all autonomous options in robotInit
	private AutoSimple simpleAuto;
	private AutoTwoBall twoBallAuto;
	private AutoPIDTest pidTest;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {

		this.drivetrainSubsystem = new DrivetrainSubsystem();
		this.shooterSubsystem = new ShooterSubsystem();
		this.indexerSubsystem = new IndexerSubsystem(shooterSubsystem);
		this.intakeSubsystem = new IntakeSubsystem(indexerSubsystem);
		this.climberSubsystem = new ClimberSubsystem();

		// SmartDashboard Stuff
		SmartDashboard.putNumber("p", .66);
		double p = SmartDashboard.getNumber("p", .66);
		SmartDashboard.putNumber("i", 0);
		double i = SmartDashboard.getNumber("i", 0);
		SmartDashboard.putNumber("d", .025);
		double d = SmartDashboard.getNumber("d", .025);
		SmartDashboard.putNumber("thetaP", 5);
		double thetaP = SmartDashboard.getNumber("thetaP", 5);
		SmartDashboard.putNumber("maxVel", 5);
		double maxVel = SmartDashboard.getNumber("maxVel", 5);
		SmartDashboard.putNumber("maxAccel", 3);
		double maxAccel = SmartDashboard.getNumber("maxAccel", 3);

		// Autonomous options
		this.simpleAuto = new AutoSimple(drivetrainSubsystem, shooterSubsystem, intakeSubsystem, indexerSubsystem);
		this.twoBallAuto = new AutoTwoBall(drivetrainSubsystem, shooterSubsystem, indexerSubsystem, intakeSubsystem);
		this.pidTest = new AutoPIDTest(drivetrainSubsystem, p, i, d, thetaP, maxVel, maxAccel);

		autonomousChooser.setDefaultOption("Simple Auto", simpleAuto);
		autonomousChooser.addOption("Two Ball Auto Right", twoBallAuto);
		autonomousChooser.addOption("PID Test Auto", pidTest);
		SmartDashboard.putData(autonomousChooser);

		// Driver button bindings
		Controllers.resetRobotOrientation.whenPressed(drivetrainSubsystem.resetGyro);
		Controllers.driveRobotOrientationButton.toggleWhenPressed(drivetrainSubsystem.roboOrientedDrive);
		Controllers.slewButton.toggleWhenPressed(drivetrainSubsystem.slewDrive);
		Controllers.climbButton.toggleWhenPressed(climberSubsystem.climbUp);
		Controllers.autoIntakeStartButton.whenPressed(intakeSubsystem.automaticIntake.withTimeout(2));

		// Operator button bindings
		Controllers.indexWrongBallOutButton.whileHeld(indexerSubsystem.wrongBallEject);
		Controllers.intakeButton.whileHeld(new Intake(intakeSubsystem, indexerSubsystem));
		Controllers.shootHighAutomaticButton.whenPressed(indexerSubsystem.readyIndex.withTimeout(Indexer.readyIndexForShoot).andThen(shooterSubsystem.readyForHighShot));
		Controllers.shootHighAutomaticButton.whenReleased(indexerSubsystem.outputBalls.withTimeout(4));

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
