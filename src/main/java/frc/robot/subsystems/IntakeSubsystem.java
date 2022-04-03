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

  CANSparkMax intakeMotor;
  DoubleSolenoid intakeSolenoid;
  //DigitalInput intakeSensor, ballOneSensor, ballTwoSensor;
  //DeployState intakeDeployState;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, Constants.intakeSolenoidIn, Constants.intakeSolenoidOut);
    intakeSolenoid.set(Value.kReverse);
    //intakeSensor = new DigitalInput(Constants.intakeSensorID);
    //ballOneSensor = new DigitalInput(Constants.ballOneSensorID);
    //ballTwoSensor = new DigitalInput(Constants.ballTwoSensorID);
  }

  public void intakeForward() {
    intakeMotor.set(Constants.intakeForward);
  }

  public void intakeBackward() {
    intakeMotor.set(Constants.intakeBack);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }

  public void intakeUp() {
    intakeMotor.set(0);
    intakeSolenoid.set(Value.kReverse);
    
  }

  public void intakeDown() {
    intakeSolenoid.set(Value.kForward);;
  }

  /*
  public boolean pullInBall() {
    boolean exitNow = false;
    while (exitNow == false) {
      // Create off conditions
      if (!ballOneSensor.get() && !ballTwoSensor.get()) {
        stopIndexerMotor();
        exitNow = true;
      }
      else if (intakeSensor.get()  && ballOneSensor.get() && ballTwoSensor.get()) {
        stopIndexerMotor();
      }
      else if (intakeSensor.get() && ballOneSensor.get() && !ballTwoSensor.get()) {
        indexBallsBack(Constants.indexSpeedBack);
      }
      else if (intakeSensor.get() && !ballOneSensor.get() && ballTwoSensor.get()) {
        stopIndexerMotor();
      }
      else if (!intakeSensor.get() && ballOneSensor.get() && ballTwoSensor.get()) {
        indexBallsForward(Constants.indexSpeedForward);
      }
      else if (!intakeSensor.get() && !ballOneSensor.get() && ballTwoSensor.get()) {
        indexBallsForward(Constants.indexSpeedForward);
      }
    }
    
    return true;
  }
  

  public boolean topBallStatus() {
    return ballTwoSensor.get();
  }

  public boolean bottomBallStatus() {
    return ballOneSensor.get();
  }

  public boolean intakeBallStatus() {
    return intakeSensor.get();
  }

  
  public Boolean intakeDeployState() {
    if (intakeDeployState == DeployState.DEPLOYED) {
      return true;
    }
    else {
      return false;
    } 
  }
  */


  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Top Sensor Status", topBallStatus());
    //SmartDashboard.putBoolean("Bottom Sensor Status", bottomBallStatus());
    //SmartDashboard.putBoolean("Intake Sensor Status", intakeBallStatus());
    //SmartDashboard.putBoolean("Deploy State", intakeDeployState());
  }
}
