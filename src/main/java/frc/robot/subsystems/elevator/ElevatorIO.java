package frc.robot.subsystems.elevator;

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

  default void setElevatorSetpointFF(double position) {
    throw new UnsupportedOperationException("setElevatorSetpointFF is not implemented");
  }

  class ElevatorIOInputs {

    // Are the motors connected to the CAN bus? //
    public boolean elevatorLeadMotorConnected;
    public boolean elevatorFollowMotorConnected;

    // Left Elevator Motor data //
    public double elevatorLeadMotorVoltage;
    public double elevatorLeadMotorDutyCycle;
    public double elevatorLeadMotorCurrent;
    public double elevatorLeadMotorTemperature;
    public double elevatorLeadMotorPosition;
    public double elevatorLeadMotorVelocity;

    // Right Elevator Motor data //
    public double elevatorFollowMotorVoltage;
    public double elevatorFollowMotorDutyCycle;
    public double elevatorFollowMotorCurrent;
    public double elevatorFollowMotorTemperature;

    // Is the current limit tripped? //
    public boolean elevatorCurrentLimitTripped;

    // Is the elevator at the slow down threshold? //
    public boolean elevatorAtSlowDownThreshold;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}
}
