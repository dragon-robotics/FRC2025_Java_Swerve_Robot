package frc.robot.subsystems.coral;

import lombok.Getter;
import lombok.Setter;

public interface CoralIO {
  default void setIntakeMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeMotorVoltage is not implemented");
  }

  default void setIntakeMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeMotorPercentage is not implemented");
  }

  class CoralIOInputs {

    // Are the motors connected to the CAN bus? //
    @Getter @Setter
    private boolean intakeMotorConnected;

    // Motor data //
    @Getter @Setter
    private double intakeMotorVoltage;
    @Getter @Setter
    private double intakeMotorCurrent;
    @Getter @Setter
    private double intakeMotorTemperature;

    // Is the beam break tripped? //
    @Getter @Setter
    private boolean beamBreakTripped;

    // Is the current limit tripped? //
    @Getter @Setter
    private boolean intakeCurrentLimitTripped;
  }

  default void updateInputs(CoralIOInputs inputs) {}
}
