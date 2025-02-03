// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import java.util.function.BooleanSupplier;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // @TODO: Create Elastic Tabs for the Arm Subsystem //

  // Algae Subsystem States //
  public enum WantedState {
    IDLE,
    HOME,
    COLLECT,
    HOLD,
    EJECT,
    OFF
  }

  public enum SystemState {
    IDLING,
    AT_HOME,
    COLLECTING,
    HOLDING,
    EJECTING,
    OFF
  }

  private AlgaeIO m_algaeIO;
  private AlgaeIOInputs m_algaeIOInputs = new AlgaeIOInputs();

  private WantedState m_wantedState = WantedState.IDLE;
  private SystemState m_systemState = SystemState.IDLING;
  
  public AlgaeSubsystem(AlgaeIO algaeIO) {
    m_algaeIO = algaeIO;
  }
  
  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return m_algaeIOInputs.intakeCurrentLimitTripped;
  }

  /**
   * Set the wanted state of the arm
   * @param wantedState
   */
  public void setWantedState(WantedState wantedState) {
    m_wantedState = wantedState;
  }

  /**
   * Handle the state transition
   * @return the new state
   */
  private SystemState handleStateTransition() {
    return switch (m_wantedState) {
      case OFF -> SystemState.OFF;
      case EJECT -> SystemState.EJECTING;
      case HOLD -> SystemState.HOLDING;
      case COLLECT -> {
        if (isCurrentLimitTripped()) {
          yield SystemState.HOLDING;
        }
        yield SystemState.COLLECTING;
      }
      default -> SystemState.AT_HOME;
    };
  }

  /**
   * Handles the action of the intake and the arm motors for each state
   */
  public void handleMotors(double armSetpoint, double intakeSpeed) {
    m_algaeIO.setArmSetpoint(armSetpoint);
    m_algaeIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_algaeIO.updateInputs(m_algaeIOInputs);
    
    // Get the new state //
    SystemState newState = handleStateTransition();
    if(newState != m_systemState) {
      m_systemState = newState;
    }

    // Stop moving when the robot is disabled //
    if (DriverStation.isDisabled()) {
      m_systemState = SystemState.IDLING;
    }

    // Handle the state transition //
    // set voltages based on state
    switch (m_systemState) {
      case EJECTING:
        // Hold the arm at the ejecting position and spin the intake to eject the algae //
        handleMotors(PROCESSOR_OUTTAKE_GOAL, OUTTAKE_SPEED);       
        break;
      case COLLECTING:
        // Hold the arm at the collecting position and spin the intake to collect the algae //  
        handleMotors(INTAKE_GOAL, INTAKE_SPEED);
        break;
      case HOLDING:
        // Hold the arm at the holding position and stop the intake from spinning //
        handleMotors(HOLD_GOAL, 0);
        break;
      case AT_HOME:
        // Hold the arm at the home position and stop the intake from spinning //
        handleMotors(HOME_GOAL, 0);
        break;
      case IDLING:
        // Hold the arm at the home position //
        m_algaeIO.setArmMotorVoltage(0);

        // Stop the intake motor from spinning //
        m_algaeIO.setIntakeMotorVoltage(0);
        break;
      case OFF:
        break;
      default:
        // Hold the arm at the home position and stop the intake from spinning //
        handleMotors(HOME_GOAL, 0);
        break;
      }
  }
}
