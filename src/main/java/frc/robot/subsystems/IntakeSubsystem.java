// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public IntakeSubsystem() {
    Constants.intakeSolenoid.set(Value.kReverse);
  }

  public void intakeForward() {
    Constants.intakeMotor.set(Constants.intakeForward);
  }

  public void intakeBackward() {
    Constants.intakeMotor.set(Constants.intakeBack);
  }

  public void intakeStop() {
    Constants.intakeMotor.set(0);
  }

  public void intakeUp() {
    Constants.intakeMotor.set(0);
    Constants.intakeSolenoid.set(Value.kReverse);
    
  }

  public void intakeDown() {
    Constants.intakeSolenoid.set(Value.kForward);;
  }


  @Override
  public void periodic() {
  }
}
