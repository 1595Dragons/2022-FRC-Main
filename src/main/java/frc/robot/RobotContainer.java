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
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IndexControl;
import frc.robot.commands.IndexWrongBallOut;
import frc.robot.commands.OutputBallsToShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.ReadyIndex;
import frc.robot.commands.ReadyShooterHigh;
import frc.robot.commands.SecondaryDriveCommand;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoRightTwoBall;
import frc.robot.commands.AutoSimple;
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
  
  public static final XboxController m_driver = new XboxController(0);
  public static final XboxController m_operator = new XboxController(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  public RobotContainer() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
            () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    m_indexerSubsystem.setDefaultCommand(new IndexControl(m_indexerSubsystem));
    m_climberSubsystem.setDefaultCommand(new ClimbDown(m_climberSubsystem));

    //SmartDashboard Stuff
    m_chooser.setDefaultOption("Simple Auto", new AutoSimple(m_drivetrainSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_indexerSubsystem));
    m_chooser.addOption("Two Ball Auto Right", new AutoRightTwoBall(m_drivetrainSubsystem, m_shooterSubsystem, m_indexerSubsystem, m_intakeSubsystem));
    SmartDashboard.putData(m_chooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    // Driver button bindings
    JoystickButton resetRobotOrientation = new JoystickButton(m_driver, OIConstants.backButton);
    resetRobotOrientation.whenPressed(new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()));
    
    JoystickButton driveSlowButton = new JoystickButton(m_driver, OIConstants.leftButtonJoystick);
    driveSlowButton.toggleWhenPressed(new SecondaryDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_driver.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
      () -> -modifyAxis(m_driver.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveNormal,
      () -> -modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.driveNormal));

    JoystickButton climbUpButton = new JoystickButton(m_driver, OIConstants.xButton);
    climbUpButton.toggleWhenPressed(new ClimbUp(m_climberSubsystem));

    JoystickButton autoIntakeStartButton = new JoystickButton(m_driver, OIConstants.aButton);
    autoIntakeStartButton.whenPressed(new AutoIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(2));

    
    // Operator button bindings

    JoystickButton indexWrongBallOutButton = new JoystickButton(m_operator, OIConstants.leftBumper);
    indexWrongBallOutButton.whileHeld(new IndexWrongBallOut(m_indexerSubsystem));
  
    JoystickButton intakeButton = new JoystickButton(m_operator, OIConstants.rightBumper);
    intakeButton.whileHeld(new Intake(m_intakeSubsystem, m_indexerSubsystem));
 
    JoystickButton shootHighAutomaticButton = new JoystickButton(m_operator, OIConstants.aButton);
    shootHighAutomaticButton.whenPressed(new ReadyIndex(m_indexerSubsystem, m_shooterSubsystem).withTimeout(Constants.readyIndexForShoot).andThen(
      new ReadyShooterHigh(m_indexerSubsystem, m_shooterSubsystem)));
    shootHighAutomaticButton.whenReleased(new OutputBallsToShoot(m_shooterSubsystem, m_indexerSubsystem).withTimeout(2));

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
