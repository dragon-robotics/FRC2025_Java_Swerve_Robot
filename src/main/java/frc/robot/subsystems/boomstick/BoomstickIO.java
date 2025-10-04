package frc.robot.subsystems.boomstick;

import lombok.Getter;
import lombok.Setter;

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
    @Getter @Setter private boolean armMotorConnected;

    // Left Arm Motor data //
    @Getter @Setter private double armMotorVoltage;
    @Getter @Setter private double armMotorDutyCycle;
    @Getter @Setter private double armMotorCurrent;
    @Getter @Setter private double armMotorTemperature;
    @Getter @Setter private double armMotorPosition;
    @Getter @Setter private double armMotorVelocity;
  }

  default void updateInputs(BoomstickIOInputs inputs) {}
}
