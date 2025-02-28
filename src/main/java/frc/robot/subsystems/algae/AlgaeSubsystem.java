// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // @TODO: Create Elastic Tabs for the Arm Subsystem //

  // Algae Subsystem States //
  public enum AlgaeState {
    IDLE,
    HOME,
    INTAKE,
    DEALGAE,
    HOLD,
    SCORE,
  }

  private AlgaeIO m_algaeIO;
  private AlgaeState m_algaeState;
  private AlgaeIOInputs m_algaeIOInputs;

  private boolean m_hasAlgae;
  
  public AlgaeSubsystem(AlgaeIO algaeIO) {
    m_algaeIO = algaeIO;
    m_algaeIOInputs = new AlgaeIOInputs();
    m_algaeState = AlgaeState.IDLE;
    m_hasAlgae = false; // The robot initially has no algae
  }
  
  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return m_algaeIOInputs.intakeCurrentLimitTripped;
  }

  /**
   * Get whether the algae intake has an algae
   */
  public boolean hasAlgae() {
    return m_hasAlgae;
  }

  /**
   * Set whether the algae intake has a algae
   * @param hasAlgae
   */
  public void setHasAlgae(boolean hasAlgae) {
    m_hasAlgae = hasAlgae;
  }

  /**
   * Set the wanted state of the arm
   * @param wantedState
   */
  public void setAlgaeState(AlgaeState wantedAlgaeState) {

    m_algaeState = wantedAlgaeState;

    switch(m_algaeState){
      case INTAKE:
        handleMotors(ARM_INTAKE_GOAL, INTAKE_SPEED);
        break;
      case DEALGAE:
        handleMotors(ARM_DEALGAE_GOAL, DEALGAE_SPEED);
        break;
      case HOLD:
        handleMotors(ARM_HOLD_GOAL, 0);
        break;
      case SCORE:
        handleMotors(ARM_PROCESSOR_OUTTAKE_GOAL, OUTTAKE_SPEED);
        break;
      case HOME:
        handleMotors(ARM_HOME_GOAL, 0);
        break;
      case IDLE:
        m_algaeIO.setIntakeMotorPercentage(0);
        m_algaeIO.setArmMotorPercentage(0);
        break;
    }
  }

  public void setAlgaeIntakeMotorSpeed(double speed) {
    m_algaeIO.setIntakeMotorPercentage(speed);
  }

  public void setAlgaeArmMotorSpeed(double speed) {
    m_algaeIO.setArmMotorPercentage(speed);
  }

  /**
   * Handles the action of the intake and the arm motors for each state
   */
  private void handleMotors(double armSetpoint, double intakeSpeed) {
    m_algaeIO.setArmSetpoint(armSetpoint);
    m_algaeIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_algaeIO.updateInputs(m_algaeIOInputs);
  }
}
