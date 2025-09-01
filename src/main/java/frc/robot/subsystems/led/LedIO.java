// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

/** Add your docs here. */
public interface LedIO {

  default void setLedColor(double ledValue) {
    throw new UnsupportedOperationException("setLedColor is not implemented");
  }

  class LedIOInputs {

    // Is the LED Controller connected? //
    public boolean ledControllerConnected;

    // LED controller data //
    public double curLedValue;
  }

  default void updateInputs(LedIOInputs inputs) {}
}
