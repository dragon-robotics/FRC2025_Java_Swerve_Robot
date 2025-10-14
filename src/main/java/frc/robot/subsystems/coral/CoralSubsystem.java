// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import static frc.robot.Constants.CoralSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;

public class CoralSubsystem extends SubsystemBase {

  // Coral Subsystem States //
  public enum CoralState {
    IDLE,
    INTAKE,
    SLOW_INTAKE,
    SLOWER_INTAKE,
    HOLD,
    SCORE,
    REVERSE,
    SLOW_REVERSE
  }

  private CoralIO coralIO;
  private CoralState coralState;
  private CoralIOInputs coralIOInputs;

  private boolean hasCoral;

  /** Creates a new IntakeSubsystem. */
  public CoralSubsystem(CoralIO coralIO) {
    this.coralIO = coralIO;
    coralIOInputs = new CoralIOInputs();
    coralState = CoralState.IDLE;
    hasCoral = false; // The robot initially has no coral
  }

  /** Get whether the coral intake has a coral */
  public boolean hasCoral() {
    return hasCoral;
  }

  /**
   * Set whether the coral intake has a coral
   *
   * @param hasCoral
   */
  public void setHasCoral(boolean hasCoral) {
    this.hasCoral = hasCoral;
  }

  public boolean isBeamBreakTripped() {
    return coralIOInputs.isBeamBreakTripped();
  }

  /**
   * Set the state of the coral intake
   *
   * @param wantedCoralState
   */
  public void setCoralState(CoralState wantedCoralState) {

    coralState = wantedCoralState;

    switch (coralState) {
      case INTAKE:
        coralIO.setIntakeMotorVoltage(INTAKE_VOLTAGE);
        break;
      case SLOW_INTAKE:
        coralIO.setIntakeMotorVoltage(SLOW_INTAKE_VOLTAGE);
        break;
      case SLOWER_INTAKE:
        coralIO.setIntakeMotorVoltage(SLOWER_INTAKE_VOLTAGE);
        break;
      case HOLD:
        coralIO.setIntakeMotorVoltage(HOLD_VOLTAGE);
        break;
      case SCORE:
        coralIO.setIntakeMotorVoltage(OUTTAKE_VOLTAGE);
        break;
      case REVERSE:
        coralIO.setIntakeMotorVoltage(REVERSE_VOLTAGE);
        break;
      case SLOW_REVERSE:
        coralIO.setIntakeMotorVoltage(SLOW_REVERSE_VOLTAGE);
        break;
      case IDLE:
      default:
        coralIO.setIntakeMotorVoltage(HOLD_VOLTAGE);
        break;
    }
  }

  /** Set the motor speeds manually */
  public void setCoralMotorSpeeds(double intakeSpeed) {
    coralIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs
    coralIO.updateInputs(coralIOInputs);
  }
}
