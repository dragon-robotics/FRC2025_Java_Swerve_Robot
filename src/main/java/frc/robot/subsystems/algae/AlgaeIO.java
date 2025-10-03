package frc.robot.subsystems.algae;

import lombok.Getter;
import lombok.Setter;

public interface AlgaeIO {

  default void setArmMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setArmMotorVoltage is not implemented");
  }

  default void setArmMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setArmMotorPercentage is not implemented");
  }

  default void setArmSetpoint(double setpoint) {
    throw new UnsupportedOperationException("setArmSetpoint is not implemented");
  }

  default void setIntakeMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeMotorVoltage is not implemented");
  }

  default void setIntakeMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeMotorPercentage is not implemented");
  }

  class AlgaeIOInputs {

    // Are the motors connected to the CAN bus? //
    @Getter @Setter
    private boolean armMotorConnected;
    @Getter @Setter
    private boolean intakeMotorConnected;

    // Arm Motor data //
    @Getter @Setter
    private double armMotorVoltage;
    @Getter @Setter
    private double armMotorDutyCycle;
    @Getter @Setter
    private double armMotorCurrent;
    @Getter @Setter
    private double armMotorTemperature;
    @Getter @Setter
    private double armMotorPosition;
    @Getter @Setter
    private double armMotorVelocity;

    // Intake motor data //
    @Getter @Setter
    private double intakeMotorVoltage;
    @Getter @Setter
    private double intakeMotorDutyCycle;
    @Getter @Setter
    private double intakeMotorCurrent;
    @Getter @Setter
    private double intakeMotorTemperature;

    // Is the current limit tripped? //
    @Getter @Setter
    private boolean intakeCurrentLimitTripped;
  }

  default void updateInputs(AlgaeIOInputs inputs) {}
}
