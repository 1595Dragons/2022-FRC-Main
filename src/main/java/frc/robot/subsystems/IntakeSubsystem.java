// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax intakeMotor;
  DoubleSolenoid intakeSolenoid;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.intakeSolenoidIn, Constants.intakeSolenoidOut);
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeForward(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeBackward(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
    intakeSolenoid.set(Value.kForward);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public void intakeUp() {
    intakeSolenoid.set(Value.kReverse);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
