package frc.robot.subsystems.coral;

public interface CoralIO {
  default void setIntakeMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeMotorVoltage is not implemented");
  }

  default void setIntakeMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeMotorPercentage is not implemented");
  }

  class CoralIOInputs {

    // Are the motors connected to the CAN bus? //
    public boolean intakeMotorConnected;

    // Motor data //
    public double intakeMotorVoltage;
    public double intakeMotorCurrent;
    public double intakeMotorTemperature;

    // Is the beam break tripped? //
    public boolean beamBreakTripped;

    // Is the current limit tripped? //
    public boolean intakeCurrentLimitTripped;

    // // Is the coral in the robot ? //
    // public boolean isCoralInRobot;
  }

  default void updateInputs(CoralIOInputs inputs) {}
}
