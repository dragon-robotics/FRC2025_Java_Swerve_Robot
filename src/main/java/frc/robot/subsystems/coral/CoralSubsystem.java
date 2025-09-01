// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import static frc.robot.Constants.CoralSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;

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
    REVERSE,
    FORWARD
  }

  private HoldState m_holdState = HoldState.REVERSE;
  private double m_forwardEndTime = 0.0;

  private CoralIO m_coralIO;
  private CoralState m_coralState;
  private CoralIOInputs m_coralIOInputs;

  private boolean m_hasCoral;

  /** Creates a new IntakeSubsystem. */
  public CoralSubsystem(CoralIO coralIO) {
    m_coralIO = coralIO;
    m_coralIOInputs = new CoralIOInputs();
    m_coralState = CoralState.IDLE;
    m_hasCoral = false; // The robot initially has no coral
  }

  /** Get whether the coral intake has a coral */
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
        m_coralIO.setIntakeMotorPercentage(INTAKE_SPEED);
        break;
      case SLOW_INTAKE:
        m_coralIO.setIntakeMotorPercentage(SLOW_INTAKE_SPEED);
        break;
      case SLOWER_INTAKE:
        m_coralIO.setIntakeMotorPercentage(0.08);
        break;
      case HOLD:
        // switch (m_holdState) {
        //   case REVERSE:
        //     // Reverse the intake until beam break detected
        //     m_coralIO.setIntakeMotorPercentage(SLOW_REVERSE_SPEED);
        //     if (m_coralIOInputs.beamBreakTripped) {
        //       // Coral hit the beam break, switch to forward for 0.1 seconds
        //       m_holdState = HoldState.FORWARD;
        //       m_forwardEndTime = Timer.getFPGATimestamp() + 0.1;
        //     }
        //     break;

        //   case FORWARD:
        //     // Run intake forward for a short time
        //     m_coralIO.setIntakeMotorPercentage(-SLOW_REVERSE_SPEED);
        //     if (Timer.getFPGATimestamp() >= m_forwardEndTime) {
        //       // 0.1 seconds elapsed, switch back to reverse
        //       m_holdState = HoldState.REVERSE;
        //     }
        //     break;
        // }
        m_coralIO.setIntakeMotorPercentage(0);
        break;
      case SCORE:
        m_coralIO.setIntakeMotorPercentage(OUTTAKE_SPEED);
        break;
      case REVERSE:
        m_coralIO.setIntakeMotorPercentage(REVERSE_SPEED);
        break;
      case SLOW_REVERSE:
        m_coralIO.setIntakeMotorPercentage(SLOW_REVERSE_SPEED);
        break;
      case IDLE:
      default:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
    }
  }

  /** Set the motor speeds manually */
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
