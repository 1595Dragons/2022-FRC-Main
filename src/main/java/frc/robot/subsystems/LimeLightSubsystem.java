// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimeLightSubsystem extends SubsystemBase {

  private NetworkTable limelightTable;

  private boolean isConnected;
  private long lastUpdatedTime;
  private long lastTimeChecked;

  public LimeLightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    initializeLimelight();

    isConnected = true;
  }

  private void initializeLimelight() {
    setLedMode(3);
    setCamMode(0);
  }

  public boolean hasValidTarget() {
    return limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  public double getTargetHorizontalOffset() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTargetVerticalOffest() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTargetArea() {
    return limelightTable.getEntry("ta").getDouble(0.0);
  }

  public double getTargetDistance() {
    if (!hasValidTarget())
      return 0.0;

    double angleToGoalDegrees = Constants.Limelight.MOUNTED_ANGLE + getTargetVerticalOffest();
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    return ((Constants.Limelight.TARGET_HEIGHT - Constants.Limelight.LENS_HEIGHT)
    / Math.tan(angleToGoalRadians));
  }

  public double getCamMode(double defaultValue) {
    return limelightTable.getEntry("camMode").getDouble(defaultValue);
  }
  public double getLedMode(double defaultValue) {
    return limelightTable.getEntry("ledMode").getDouble(defaultValue);
  }

  public boolean getConnected() {
    return isConnected;
  }
  
  public void setCamMode(double camMode) {
    limelightTable.getEntry("camMode").setNumber(camMode);
  }
  
  public void setLedMode(double ledMode) {
    limelightTable.getEntry("ledMode").setNumber(ledMode);
  }

  public void setPiPMode(double pipMode) {
    limelightTable.getEntry("stream").setNumber(pipMode);
  }

  @Override
  public void periodic() {
    isConnected = true;

    long anyLastUpdated = limelightTable.getEntry("ta").getLastChange();
    long current = System.nanoTime();

    if (anyLastUpdated == lastUpdatedTime) {
        if (current - lastTimeChecked > 1_000_000_000) {
            isConnected = false;
        }
    }

    else {
        lastUpdatedTime = anyLastUpdated;
        lastTimeChecked = current;
    }
  }
}
