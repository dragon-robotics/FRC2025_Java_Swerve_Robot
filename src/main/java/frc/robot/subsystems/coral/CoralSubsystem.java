// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.subsystems.coral.CoralIO;
import frc.robot.subsystems.coral.CoralIO.CoralIOInputs;
import frc.robot.Constants.GeneralConstants;
import static frc.robot.Constants.CoralSubsystemConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Intake Subsystem //

  // Coral Subsystem States //
  public enum WantedState {
    IDLE,
    INTAKE,
    SLOW_INTAKE,
    HOLD,
    EJECT,
    OFF
  }

  public enum SystemState {
    IDLING,
    INTAKING,
    SLOW_INTAKING,
    HOLDING,
    EJECTING,
    OFF
  }

  private CoralIO m_coralIO;
  private CoralIOInputs m_coralIOInputs = new CoralIOInputs();

  private WantedState m_wantedState = WantedState.IDLE;
  private SystemState m_systemState = SystemState.IDLING;

  public boolean m_hasCoral = false;

  /**
   * Creates a new IntakeSubsystem.
   */
  public CoralSubsystem(CoralIO coralIO) {
    m_coralIO = coralIO;
  }

  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isBeamBreakTripped() {
    return m_coralIOInputs.beamBreakTripped;
  }

  /**
   * Get whether the coral intake has a coral
   */
  public boolean hasCoral() {
    return m_hasCoral;
  }

  /**
   * Set the wanted state of the coral intake
   * @param wantedState
   */
  public void setWantedState(WantedState wantedState) {
    m_wantedState = wantedState;
  }

  /**
   * Get the current state of the coral intake
   * @return m_systemState
   */
  public SystemState getSystemState() {
    return m_systemState;
  }

  /**
   * Handle the state transition
   * @return the new state
   */
  private SystemState handleStateTransition() {
    return switch (m_wantedState) {
      case OFF -> SystemState.OFF;
      case EJECT -> SystemState.EJECTING;
      case HOLD -> {
        if (isBeamBreakTripped()) {
          yield SystemState.HOLDING;
        }
        yield SystemState.IDLING;
      }
      case SLOW_INTAKE -> {
        if (!isBeamBreakTripped()) {
          yield SystemState.HOLDING;
        }
        yield SystemState.SLOW_INTAKING;
      }
      case INTAKE -> {
        if (isBeamBreakTripped()) {
          yield SystemState.SLOW_INTAKING;
        }
        yield SystemState.INTAKING;
      }
      default -> SystemState.IDLING;
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_coralIO.updateInputs(m_coralIOInputs);
    
    // Get the new state //
    SystemState newState = handleStateTransition();
    if(newState != m_systemState) {
      m_systemState = newState;
    }

    // Stop moving when the robot is disabled //
    if (DriverStation.isDisabled()) {
      m_systemState = SystemState.IDLING;
    }

    // Handle the state transitions //
    switch (m_systemState) {
      case IDLING:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
      case INTAKING:
        m_coralIO.setIntakeMotorPercentage(INTAKE_SPEED);
        break;
      case SLOW_INTAKING:
        m_coralIO.setIntakeMotorPercentage(SLOW_INTAKE_SPEED);
        setWantedState(WantedState.SLOW_INTAKE); // Slowly intake until the beam break is untripped again
        break;
      case HOLDING:
        m_coralIO.setIntakeMotorPercentage(0);
        m_hasCoral = true;
        break;
      case EJECTING:
        m_coralIO.setIntakeMotorPercentage(OUTTAKE_SPEED);
        m_hasCoral = false;
        break;
      case OFF:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
      default:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
    }
  }
}
