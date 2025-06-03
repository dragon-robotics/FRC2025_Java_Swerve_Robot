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

  private LedIO m_ledIO;
  private LedState m_ledState;
  private LedIOInputs m_ledIOInputs;

  /** Creates a new LedSubsystem. */
  public LedSubsystem(LedIO ledIO) {
    m_ledIO = ledIO;
    m_ledState = LedState.IDLE;
    m_ledIOInputs = new LedIOInputs();
  }

  public void setLedState(LedState ledState) {
    m_ledState = ledState;
    switch (m_ledState) {
      case IDLE:
        m_ledIO.setLedColor(LEDConstants.GRADIENT_1_AND_2); // Set to idle color
        break;
      case AUTO_NOT_ALIGNED:
        m_ledIO.setLedColor(LEDConstants.SHOT_RED); // Set to auto not aligned color
        break;
      case AUTO_ALIGNED:
        m_ledIO.setLedColor(LEDConstants.GREEN); // Set to auto aligned color
        break;
      case READY_TO_SCORE:
        m_ledIO.setLedColor(LEDConstants.COLOR1_STROBE); // Set to ready to score color
        break;
      case INTAKE:
        m_ledIO.setLedColor(LEDConstants.WHITE); // Set to intake color
        break;
      case CORAL_ONLY_IN_BOT:
        m_ledIO.setLedColor(LEDConstants.BLUE_VIOLET); // Set to coral in bot color
        break;
      case ALGAE_ONLY_IN_BOT:
        m_ledIO.setLedColor(LEDConstants.ORANGE); // Set to algae in bot color
        break;
      case CORAL_AND_ALGAE_IN_BOT:
        m_ledIO.setLedColor(LEDConstants.SKY_BLUE); // Set to algae in bot color
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ledIO.updateInputs(m_ledIOInputs);
  }
}
