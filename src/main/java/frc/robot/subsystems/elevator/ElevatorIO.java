package frc.robot.subsystems.elevator;

import lombok.Getter;
import lombok.Setter;

public interface ElevatorIO {
  default void seedElevatorMotorEncoderPosition(double position) {
    throw new UnsupportedOperationException("seedElevatorMotorEncoderPosition is not implemented");
  }

  default void setElevatorMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setElevatorMotorVoltage is not implemented");
  }

  default void setElevatorMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setElevatorMotorPercentage is not implemented");
  }

  default void setElevatorMotorSetpoint(double position) {
    throw new UnsupportedOperationException("setElevatorMotorPosition is not implemented");
  }

  default double getElevatorSetpoint() {
    throw new UnsupportedOperationException("getElevatorSetpoint is not implemented");
  }

  class ElevatorIOInputs {

    // Are the motors connected to the CAN bus? //
    @Getter @Setter
    private boolean elevatorLeadMotorConnected;
    @Getter @Setter
    private boolean elevatorFollowMotorConnected;

    // Left Elevator Motor data //
    @Getter @Setter
    private double elevatorLeadMotorVoltage;
    @Getter @Setter
    private double elevatorLeadMotorDutyCycle;
    @Getter @Setter
    private double elevatorLeadMotorCurrent;
    @Getter @Setter
    private double elevatorLeadMotorTemperature;
    @Getter @Setter
    private double elevatorLeadMotorPosition;
    @Getter @Setter
    private double elevatorLeadMotorVelocity;

    // Right Elevator Motor data //
    @Getter @Setter
    private double elevatorFollowMotorVoltage;
    @Getter @Setter
    private double elevatorFollowMotorDutyCycle;
    @Getter @Setter
    private double elevatorFollowMotorCurrent;
    @Getter @Setter
    private double elevatorFollowMotorTemperature;

    // Is the current limit tripped? //
    @Getter @Setter
    private boolean elevatorCurrentLimitTripped;

    // Is the elevator at the slow down threshold? //
    @Getter @Setter
    private boolean elevatorAtSlowDownThreshold;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}
}
