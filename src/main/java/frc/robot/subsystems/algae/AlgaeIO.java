package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

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

  default void setArmSetpointFF(double setpoint) {
    throw new UnsupportedOperationException("setArmSetpointFF is not implemented");
  }

  default void setIntakeMotorVoltage(double voltage) {
    throw new UnsupportedOperationException("setIntakeMotorVoltage is not implemented");
  }

  default void setIntakeMotorPercentage(double percentage) {
    throw new UnsupportedOperationException("setIntakeMotorPercentage is not implemented");
  }

  @AutoLog
  class AlgaeIOInputs {

    // Are the motors connected to the CAN bus? //
    public boolean armLeadMotorConnected;
    public boolean armFollowMotorConnected;
    public boolean intakeMotorConnected;

    // Left Arm Motor data //
    public double armLeadMotorVoltage;
    public double armLeadMotorDutyCycle;
    public double armLeadMotorCurrent;
    public double armLeadMotorTemperature;
    public double armLeadMotorPosition;
    public double armLeadMotorVelocity;

    // Right Arm Motor data //
    public double armFollowMotorVoltage;
    public double armFollowMotorDutyCycle;
    public double armFollowMotorCurrent;
    public double armFollowMotorTemperature;

    // Intake motor data //
    public double intakeMotorVoltage;
    public double intakeMotorDutyCycle;
    public double intakeMotorCurrent;
    public double intakeMotorTemperature;

    // Is the current limit tripped? //
    public boolean intakeCurrentLimitTripped;
  }

  default void updateInputs(AlgaeIOInputs inputs) {}
}
