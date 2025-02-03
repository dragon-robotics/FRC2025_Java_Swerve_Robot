package frc.robot.subsystems.algae;

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
        public boolean armLeftMotorConnected;
        public boolean armRightMotorConnected;
        public boolean intakeMotorConnected;

        // Motor data //
        public double armLeftMotorVoltage;
        public double armLeftMotorCurrent;
        public double armLeftMotorTemperature;
        public double armLeftMotorSetpoint;
        public double armRightMotorVoltage;
        public double armRightMotorCurrent;
        public double armRightMotorTemperature;
        public double intakeMotorVoltage;
        public double intakeMotorCurrent;
        public double intakeMotorTemperature;

        // Is the current limit tripped? //
        public boolean intakeCurrentLimitTripped;
    }

    default void updateInputs(AlgaeIOInputs inputs) {}
}
