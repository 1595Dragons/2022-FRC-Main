// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.IndexControl;
import frc.robot.commands.IndexWrongBallOut;
import frc.robot.commands.OutputBallsToShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.ReadyIndex;
import frc.robot.commands.ReadyShooterHigh;
import frc.robot.commands.Autonomous.AutoIntake;
import frc.robot.commands.Autonomous.AutoPIDTest;
import frc.robot.commands.Autonomous.AutoSimple;
import frc.robot.commands.Autonomous.AutoTwoBall;
import frc.robot.commands.Drive.FieldRelativeDrive;
import frc.robot.commands.Drive.RobotOrientedDrive;
import frc.robot.commands.Drive.SlewRatedDriveCommand;
import frc.robot.commands.ClimbDown;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new FieldRelativeDrive(
            m_drivetrainSubsystem,
            () -> -modifyAxis(Constants.driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> -modifyAxis(Constants.driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> -modifyAxis(Constants.driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    m_indexerSubsystem.setDefaultCommand(new IndexControl(m_indexerSubsystem));
    m_climberSubsystem.setDefaultCommand(new ClimbDown(m_climberSubsystem));

    //SmartDashboard Stuff
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

    m_chooser.setDefaultOption("Simple Auto", new AutoSimple(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem));
    m_chooser.addOption("Two Ball Auto", new AutoTwoBall(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    m_chooser.addOption("PID Test Auto", new AutoPIDTest(m_drivetrainSubsystem, p, i, d, thetaP, maxVel, maxAccel));
    SmartDashboard.putData(m_chooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    // Driver button bindings
    Constants.resetRobotOrientation.whenPressed(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope));

    Constants.robotOrientedButton.toggleWhenPressed(new RobotOrientedDrive(
      m_drivetrainSubsystem,
      () -> -modifyAxis(Constants.driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
      () -> -modifyAxis(Constants.driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
      () -> -modifyAxis(Constants.driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    Constants.slewRatedDriveCommandButton.toggleWhenPressed(new SlewRatedDriveCommand(m_drivetrainSubsystem,
    () -> -modifyAxis(Constants.driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
    () -> -modifyAxis(Constants.driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
    () -> -modifyAxis(Constants.driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    Constants.climbUpButton.toggleWhenPressed(new ClimbUp(m_climberSubsystem));
    Constants.autoIntakeStartButton.whenPressed(new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(2));

    // Operator button bindings
    Constants.indexWrongBallOutButton.whileHeld(new IndexWrongBallOut(m_indexerSubsystem));
    Constants.intakeButton.whileHeld(new Intake(m_intakeSubsystem, m_indexerSubsystem));
    Constants.shootHighAutomaticButton.whenPressed(new ReadyIndex(m_indexerSubsystem, m_shooterSubsystem).withTimeout(Constants.readyIndexForShoot).andThen(
      new ReadyShooterHigh(m_indexerSubsystem, m_shooterSubsystem)));
    Constants.shootHighAutomaticButton.whenReleased(new OutputBallsToShoot(m_shooterSubsystem, m_indexerSubsystem).withTimeout(4));

  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
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
}
