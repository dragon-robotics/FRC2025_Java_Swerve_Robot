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
    HOLD,
    EJECT,
  }

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
   * @param hasCoral
   */
  public void setHasCoral(boolean hasCoral) {
    m_hasCoral = hasCoral;
  }

  /**
   * Set the state of the coral intake
   * @param wantedCoralState
   */
  public void setCoralState(CoralState wantedCoralState){

    m_coralState = wantedCoralState;

    switch(m_coralState){
      case INTAKE:
        m_coralIO.setIntakeMotorPercentage(INTAKE_SPEED);
        break;
      case SLOW_INTAKE:
        m_coralIO.setIntakeMotorPercentage(SLOW_INTAKE_SPEED);
        break;
      case HOLD:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
      case EJECT:
        m_coralIO.setIntakeMotorPercentage(OUTTAKE_SPEED);
        break;
      case IDLE:
      default:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_coralIO.updateInputs(m_coralIOInputs);
  }
}
