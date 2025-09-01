package frc.robot.subsystems.boomstick;

public interface BoomstickIO {
  default void setArmMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setArmMotorVoltage is not implemented");
  }

  default void setArmMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setArmMotorPercentage is not implemented");
  }

  default void setArmSetpoint(double setpoint) {
    throw new UnsupportedOperationException("setArmSetpoint is not implemented");
  }

  class BoomstickIOInputs {

    // Are the motors connected to the CAN bus? //
    public boolean armMotorConnected;

    // Left Arm Motor data //
    public double armMotorVoltage;
    public double armMotorDutyCycle;
    public double armMotorCurrent;
    public double armMotorTemperature;
    public double armMotorPosition;
    public double armMotorVelocity;
  }

  default void updateInputs(BoomstickIOInputs inputs) {}
}
