// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class LedIORevBlinkin implements LedIO {
  // Create a Spark motor controller instance for the Rev Blinkin
  private Spark ledController;

  // The current LED value
  private double curLedValue;

  public LedIORevBlinkin() {
    // Initialize the Spark motor controller
    int ledChannel = 9; // Example channel number, change as needed
    ledController = new Spark(ledChannel);
  }

  @Override
  public void setLedColor(double ledValue) {
    curLedValue = ledValue;
    ledController.set(ledValue);
  }

  @Override
  public void updateInputs(LedIOInputs inputs) {
    inputs.setLedControllerConnected(ledController.isAlive());
    inputs.setCurLedValue(curLedValue);
  }
}
