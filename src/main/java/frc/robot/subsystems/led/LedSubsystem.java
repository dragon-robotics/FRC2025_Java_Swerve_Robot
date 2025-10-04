// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.led.LedIO.LedIOInputs;

public class LedSubsystem extends SubsystemBase {

  // LED Subsystem States //
  public enum LedState {
    IDLE,
    AUTO_NOT_ALIGNED,
    AUTO_ALIGNED,
    READY_TO_SCORE,
    INTAKE,
    CORAL_ONLY_IN_BOT,
    ALGAE_ONLY_IN_BOT,
    CORAL_AND_ALGAE_IN_BOT
  }

  private LedIO ledIO;
  private LedState ledState;
  private LedIOInputs ledIOInputs;

  /** Creates a new LedSubsystem. */
  public LedSubsystem(LedIO ledIO) {
    this.ledIO = ledIO;
    ledState = LedState.IDLE;
    ledIOInputs = new LedIOInputs();
  }

  public void setLedState(LedState ledState) {
    this.ledState = ledState;
    switch (this.ledState) {
      case IDLE:
        ledIO.setLedColor(LEDConstants.GRADIENT_1_AND_2); // Set to idle color
        break;
      case AUTO_NOT_ALIGNED:
        ledIO.setLedColor(LEDConstants.SHOT_RED); // Set to auto not aligned color
        break;
      case AUTO_ALIGNED:
        ledIO.setLedColor(LEDConstants.GREEN); // Set to auto aligned color
        break;
      case READY_TO_SCORE:
        ledIO.setLedColor(LEDConstants.COLOR1_STROBE); // Set to ready to score color
        break;
      case INTAKE:
        ledIO.setLedColor(LEDConstants.WHITE); // Set to intake color
        break;
      case CORAL_ONLY_IN_BOT:
        ledIO.setLedColor(LEDConstants.BLUE_VIOLET); // Set to coral in bot color
        break;
      case ALGAE_ONLY_IN_BOT:
        ledIO.setLedColor(LEDConstants.ORANGE); // Set to algae in bot color
        break;
      case CORAL_AND_ALGAE_IN_BOT:
        ledIO.setLedColor(LEDConstants.SKY_BLUE); // Set to algae in bot color
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledIO.updateInputs(ledIOInputs);
  }
}
