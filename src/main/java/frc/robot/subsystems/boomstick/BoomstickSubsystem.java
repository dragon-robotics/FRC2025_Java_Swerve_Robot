// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.boomstick;

import static frc.robot.Constants.BoomstickSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.boomstick.BoomstickIO.BoomstickIOInputs;

public class BoomstickSubsystem extends SubsystemBase {
  /** Creates a new BoomstickSubsystem. */

  // Boomstick Subsystem States //
  public enum BoomstickState {
    IDLE,
    HOME,
    LOW_DEALGAE,
    HIGH_DEALGAE,
  }

  private BoomstickIO boomstickIO;
  private BoomstickState boomstickState;
  private BoomstickIOInputs boomstickIOInputs;

  public BoomstickSubsystem(BoomstickIO boomstickIO) {
    this.boomstickIO = boomstickIO;
    boomstickIOInputs = new BoomstickIOInputs();
    boomstickState = BoomstickState.IDLE;
  }

  /**
   * Set the wanted state of the boomstick arm
   *
   * @param wantedState
   */
  public void setBoomstickState(BoomstickState wantedState) {

    boomstickState = wantedState;

    switch (boomstickState) {
      case IDLE:
        // Set the arm to the idle position
        boomstickIO.setArmMotorVoltage(0);
        break;
      case HOME:
        // Set the arm to the home position
        boomstickIO.setArmSetpoint(ARM_HIGH_DEALGAE_GOAL);
        break;
      case LOW_DEALGAE:
        // Set the arm to the low dealgae position
        boomstickIO.setArmSetpoint(ARM_LOW_DEALGAE_GOAL);
        break;
      case HIGH_DEALGAE:
        // Set the arm to the high dealgae position
        boomstickIO.setArmSetpoint(ARM_HIGH_DEALGAE_GOAL);
        break;
      default:
        // Set the arm to the idle position
        boomstickIO.setArmMotorVoltage(0);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs
    boomstickIO.updateInputs(boomstickIOInputs);
  }
}
