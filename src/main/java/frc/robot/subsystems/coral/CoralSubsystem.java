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
    COLLECT,
    HOLD,
    EJECT,
    OFF
  }

  public enum SystemState {
    IDLING,
    COLLECTING,
    HOLDING,
    EJECTING,
    OFF
  }

  private CoralIO m_coralIO;
  private CoralIOInputs m_coralIOInputs = new CoralIOInputs();

  private WantedState m_wantedState = WantedState.IDLE;
  private SystemState m_systemState = SystemState.IDLING;

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
        if (isBeamBreakTripped()) {
          yield SystemState.HOLDING;
        }
        yield SystemState.COLLECTING;
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
      case COLLECTING:
        m_coralIO.setIntakeMotorPercentage(INTAKE_SPEED);
        break;
      case HOLDING:
        m_coralIO.setIntakeMotorPercentage(0);
        break;
      case EJECTING:
        m_coralIO.setIntakeMotorPercentage(OUTTAKE_SPEED);
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
