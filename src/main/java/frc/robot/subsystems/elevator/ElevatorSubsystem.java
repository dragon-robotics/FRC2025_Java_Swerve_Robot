// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorSubsystemConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorSubsystem extends SubsystemBase {

  // Algae Subsystem States //
  public enum ElevatorState {
    IDLE,
    HOME,
    L1,
    L2,
    L3,
    L4,
  }

  private ElevatorIO elevatorIO;
  private ElevatorState elevatorState;
  private ElevatorIOInputs elevatorIOInputs;

  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    elevatorState = ElevatorState.IDLE;
    elevatorIOInputs = new ElevatorIOInputs();
  }

  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return elevatorIOInputs.isElevatorCurrentLimitTripped();
  }

  public boolean isAtElevatorState() {
    return switch (elevatorState) {
      case L1 -> MathUtil.isNear(elevatorIO.getElevatorSetpoint(), L1, 0.2);
      case L2 -> MathUtil.isNear(elevatorIO.getElevatorSetpoint(), L2, 0.2);
      case L3 -> MathUtil.isNear(elevatorIO.getElevatorSetpoint(), L3, 0.2);
      case L4 -> MathUtil.isNear(elevatorIO.getElevatorSetpoint(), L4, 0.2);
      case HOME -> {
        // Get the current encoder position.
        double encoderPosition = elevatorIOInputs.getElevatorLeadMotorPosition();

        // Check if the encoder is at the bottom
        boolean encoderAtBottom = MathUtil.isNear(HOME, encoderPosition, 0.01);

        // Check if the current limit is tripped
        boolean currentSpiked = elevatorIOInputs.isElevatorCurrentLimitTripped();

        yield encoderAtBottom && currentSpiked;
      }
      default -> false;
    };
  }

  public void seedElevatorMotorEncoderPosition(double position) {
    elevatorIO.seedElevatorMotorEncoderPosition(position);
  }

  /**
   * Set the height of the elevator
   *
   * @param wantedElevatorState
   */
  public void setElevatorState(ElevatorState wantedElevatorState) {

    elevatorState = wantedElevatorState;

    switch (elevatorState) {
      case L1:
        elevatorIO.setElevatorMotorSetpoint(L1);
        break;
      case L2:
        elevatorIO.setElevatorMotorSetpoint(L2);
        break;
      case L3:
        elevatorIO.setElevatorMotorSetpoint(L3);
        break;
      case L4:
        elevatorIO.setElevatorMotorSetpoint(L4);
        break;
      case HOME:
        elevatorIO.setElevatorMotorSetpoint(HOME);
        break;
      case IDLE:
        // Stop the motors if the wanted state is IDLE //
        elevatorIO.setElevatorMotorPercentage(0);
        break;
      default:
        break;
    }
  }

  /** Get the elevator state */
  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  public void setElevatorMotorSpeed(double speed) {
    elevatorIO.setElevatorMotorPercentage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update inputs
    elevatorIO.updateInputs(elevatorIOInputs);
  }
}
