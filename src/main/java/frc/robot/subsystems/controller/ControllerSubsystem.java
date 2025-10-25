// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.controller.ControllerIO.ControllerIOInputs;

public class ControllerSubsystem extends SubsystemBase {

  // Controller Subsystem States //
  public enum ControllerState {
    NO_RUMBLE,
    WEAK_RUMBLE,
    MEDIUM_RUMBLE,
    STRONG_RUMBLE
  }

  private ControllerIO controllerIO;
  private ControllerState controllerState;
  private ControllerIOInputs controllerIOInputs;

  /** Creates a new ControllerSubsystem. */
  public ControllerSubsystem(ControllerIO controllerIO) {
    this.controllerIO = controllerIO;
    controllerState = ControllerState.NO_RUMBLE;
    controllerIOInputs = new ControllerIOInputs();
  }

  public void setControllerState(ControllerState controllerState) {
    this.controllerState = controllerState;
    switch (this.controllerState) {
      case NO_RUMBLE:
        controllerIO.setControllerRumble(0.0); // No rumble
        break;
      case WEAK_RUMBLE:
        controllerIO.setControllerRumble(0.3); // Weak rumble
        break;
      case MEDIUM_RUMBLE:
        controllerIO.setControllerRumble(0.6); // Medium rumble
        break;
      case STRONG_RUMBLE:
        controllerIO.setControllerRumble(1.0); // Strong rumble
        break;
      default:
        controllerIO.setControllerRumble(0.0); // Default to no rumble
        break;
    }
  }

  public void setControllerState(ControllerState controllerState, boolean left) {
    this.controllerState = controllerState;
    switch (this.controllerState) {
      case NO_RUMBLE:
        controllerIO.setControllerRumble(0.0, left); // No rumble
        break;
      case WEAK_RUMBLE:
        controllerIO.setControllerRumble(0.3, left); // Weak rumble
        break;
      case MEDIUM_RUMBLE:
        controllerIO.setControllerRumble(0.6, left); // Medium rumble
        break;
      case STRONG_RUMBLE:
        controllerIO.setControllerRumble(1.0, left); // Strong rumble
        break;
      default:
        controllerIO.setControllerRumble(0.0, left); // Default to no rumble
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    controllerIO.updateInputs(controllerIOInputs);
  }
}