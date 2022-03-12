// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.EjectWrongBallOut;
import frc.robot.commands.IntakeBack;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.AutoCommands.PIDTestAuto;
import frc.robot.commands.AutoCommands.ThreeBallAutoLong;
import frc.robot.commands.AutoCommands.TwoBallAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final XboxController m_driver = new XboxController(0);
  private final XboxController m_operator = new XboxController(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    m_intakeSubsystem.setDefaultCommand(new IntakeUp(m_intakeSubsystem));
    m_shooterSubsystem.setDefaultCommand(new InstantCommand(() -> m_shooterSubsystem.shootStop()));

    //SmartDashboard Stuff
    m_chooser.addOption("Two Ball Auto", new TwoBallAuto(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.addOption("Three Ball Auto Long", new ThreeBallAutoLong(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem));
    m_chooser.setDefaultOption("PID Test Auto", new PIDTestAuto(m_drivetrainSubsystem));
    //m_chooser.addOption(name, object);

    SmartDashboard.putData(m_chooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    // Driver button bindings
    JoystickButton resetRobotOrientation = new JoystickButton(m_driver, OIConstants.backButton);
    resetRobotOrientation.whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));
    
    JoystickButton intakeForwardButton = new JoystickButton(m_driver, OIConstants.rightBumper);
    intakeForwardButton.whileHeld(new IntakeForward(m_intakeSubsystem));

    JoystickButton ejectWrongBallButton = new JoystickButton(m_driver, OIConstants.aButton);
    ejectWrongBallButton.whileHeld(new EjectWrongBallOut(m_intakeSubsystem));
    
    JoystickButton intakeBackwardButton = new JoystickButton(m_driver, OIConstants.leftBumper);
    intakeBackwardButton.whileHeld(new IntakeBack(m_intakeSubsystem));


    // Operator button bindings
    JoystickButton shootLowButton = new JoystickButton(m_operator, OIConstants.leftBumper);
    shootLowButton.whileHeld(new ShootLow(m_shooterSubsystem, m_intakeSubsystem));

    JoystickButton shootHighButton = new JoystickButton(m_operator, OIConstants.rightBumper);
    shootHighButton.whileHeld(new ShootHigh(m_shooterSubsystem, m_intakeSubsystem));

    JoystickButton climbUpButton = new JoystickButton(m_operator, OIConstants.aButton);
    climbUpButton.whenPressed(new ClimbUp(m_climberSubsystem));

    JoystickButton climbDownButton = new JoystickButton(m_operator, OIConstants.aButton);
    climbDownButton.whenPressed(new ClimbDown(m_climberSubsystem));


    JoystickButton driveSlowButton = new JoystickButton(m_driver, OIConstants.leftButtonJoystick);
    driveSlowButton.toggleWhenPressed(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSlow,
      () -> modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSlow,
      () -> modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveSlow));
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
