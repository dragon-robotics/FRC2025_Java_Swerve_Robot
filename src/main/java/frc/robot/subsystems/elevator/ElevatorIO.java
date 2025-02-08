package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    default void setElevatorMotorVoltage(double voltage) {
        throw new UnsupportedOperationException("setElevatorMotorVoltage is not implemented");
    }

    default void setElevatorMotorPercentage(double percentage) {
        throw new UnsupportedOperationException("setElevatorMotorPercentage is not implemented");
    }

    default void setElevatorMotorSetpoint(double position) {
        throw new UnsupportedOperationException("setElevatorMotorPosition is not implemented");
    }

    default void setElevatorSetpointFF(double position) {
        throw new UnsupportedOperationException("setElevatorSetpointFF is not implemented");
    }

    class ElevatorIOInputs {
        
        // Are the motors connected to the CAN bus? //
        public boolean elevatorLeftMotorConnected;
        public boolean elevatorRightMotorConnected;

        // Left Elevator Motor data //
        public double elevatorLeftMotorVoltage;
        public double elevatorLeftMotorDutyCycle;
        public double elevatorLeftMotorCurrent;
        public double elevatorLeftMotorTemperature;
        public double elevatorLeftMotorPosition;
        public double elevatorLeftMotorVelocity;

        // Right Elevator Motor data //
        public double elevatorRightMotorVoltage;
        public double elevatorRightMotorDutyCycle;
        public double elevatorRightMotorCurrent;
        public double elevatorRightMotorTemperature;

        // Is the current limit tripped? //
        public boolean elevatorCurrentLimitTripped;

        // Is the elevator at the slow down threshold? //
        public boolean elevatorAtSlowDownThreshold;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}
}
