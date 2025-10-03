// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // Algae Subsystem States //
  public enum AlgaeState {
    IDLE,
    HOME,
    INTAKE,
    DEALGAE,
    HOLD,
    SCORE,
  }

  private AlgaeIO algaeIO;
  private AlgaeState algaeState;
  private AlgaeIOInputs algaeIOInputs;

  private boolean hasAlgae;

  public AlgaeSubsystem(AlgaeIO algaeIO) {
    this.algaeIO = algaeIO;
    algaeIOInputs = new AlgaeIOInputs();
    algaeState = AlgaeState.IDLE;
    hasAlgae = false; // The robot initially has no algae
  }

  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return algaeIOInputs.isIntakeCurrentLimitTripped();
  }

  /** Get whether the algae intake has an algae */
  public boolean hasAlgae() {
    return hasAlgae;
  }

  /**
   * Set whether the algae intake has a algae
   *
   * @param hasAlgae
   */
  public void setHasAlgae(boolean hasAlgae) {
    this.hasAlgae = hasAlgae;
  }

  /**
   * Set the wanted state of the arm
   *
   * @param wantedState
   */
  public void setAlgaeState(AlgaeState wantedAlgaeState) {

    algaeState = wantedAlgaeState;

    switch (algaeState) {
      case INTAKE:
        handleMotors(ARM_INTAKE_GOAL, INTAKE_SPEED);
        break;
      case DEALGAE:
        handleMotors(ARM_DEALGAE_GOAL, DEALGAE_SPEED);
        break;
      case HOLD:
        handleMotors(ARM_HOLD_GOAL, 0.1);
        break;
      case SCORE:
        handleMotors(ARM_PROCESSOR_OUTTAKE_GOAL, OUTTAKE_SPEED);
        break;
      case HOME:
        handleMotors(ARM_HOME_GOAL, 0);
        break;
      case IDLE:
        algaeIO.setIntakeMotorPercentage(0);
        algaeIO.setArmMotorPercentage(0);
        break;
    }
  }

  public void setAlgaeIntakeMotorSpeed(double speed) {
    algaeIO.setIntakeMotorPercentage(speed);
  }

  public void setAlgaeArmMotorSpeed(double speed) {
    algaeIO.setArmMotorPercentage(speed);
  }

  /** Handles the action of the intake and the arm motors for each state */
  private void handleMotors(double armSetpoint, double intakeSpeed) {
    algaeIO.setArmSetpoint(armSetpoint);
    algaeIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs
    algaeIO.updateInputs(algaeIOInputs);
  }
}
