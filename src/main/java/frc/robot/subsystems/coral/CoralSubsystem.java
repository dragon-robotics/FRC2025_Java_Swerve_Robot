// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;
import static frc.robot.Constants.CoralSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Intake Subsystem //

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

  private enum HoldState {
    REVERSE, FORWARD
  }

  private HoldState m_holdState = HoldState.REVERSE;
  private double m_forwardEndTime = 0.0;

  private CoralIO m_coralIO;
  private CoralState m_coralState;
  private CoralIOInputs m_coralIOInputs;

  private boolean m_hasCoral;

  /**
   * Creates a new IntakeSubsystem.
   */
  public CoralSubsystem(CoralIO coralIO) {
    m_coralIO = coralIO;
    m_coralIOInputs = new CoralIOInputs();
    m_coralState = CoralState.IDLE;
    m_hasCoral = false; // The robot initially has no coral
  }

  /**
   * Get whether the coral intake has a coral
   */
  public boolean hasCoral() {
    return m_hasCoral;
  }

  /**
   * Set whether the coral intake has a coral
   * 
   * @param hasCoral
   */
  public void setHasCoral(boolean hasCoral) {
    m_hasCoral = hasCoral;
  }

  public boolean isBeamBreakTripped() {
    return m_coralIOInputs.beamBreakTripped;
  }

  /**
   * Set the state of the coral intake
   *
   * @param wantedCoralState
   */
  public void setCoralState(CoralState wantedCoralState) {

    m_coralState = wantedCoralState;

    switch (m_coralState) {
      case INTAKE:
        m_coralIO.setIntakeMotorVoltage(INTAKE_VOLTAGE);
        break;
      case SLOW_INTAKE:
        m_coralIO.setIntakeMotorVoltage(SLOW_INTAKE_VOLTAGE);
        break;
      case SLOWER_INTAKE:
        m_coralIO.setIntakeMotorVoltage(SLOWER_INTAKE_VOLTAGE);
        break;
      case HOLD:
        m_coralIO.setIntakeMotorVoltage(HOLD_VOLTAGE);
        break;
      case SCORE:
        m_coralIO.setIntakeMotorVoltage(OUTTAKE_VOLTAGE);
        break;
      case REVERSE:
        m_coralIO.setIntakeMotorVoltage(REVERSE_VOLTAGE);
        break;
      case SLOW_REVERSE:
        m_coralIO.setIntakeMotorVoltage(SLOW_REVERSE_VOLTAGE);
        break;
      case IDLE:
      default:
        m_coralIO.setIntakeMotorVoltage(HOLD_VOLTAGE);
        break;
    }
  }

  /**
   * Set the motor speeds manually
   */
  public void setCoralMotorSpeeds(double intakeSpeed) {
    m_coralIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs
    m_coralIO.updateInputs(m_coralIOInputs);
  }
}
