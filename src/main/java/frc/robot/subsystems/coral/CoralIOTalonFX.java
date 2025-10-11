package frc.robot.subsystems.coral;

import static frc.robot.Constants.CoralSubsystemConstants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.Units;

public class CoralIOTalonFX implements CoralIO {
  private final TalonFX intakeMotor;
  private final CANrange canRangeNear; // The CANRange closer to the entry of the end-effector
  private final CANrange canRangeFar; // The CANRange close to the exit of the end-effector
  private final CANrangeConfiguration canRangeConfigs;

  private int ntUpdateCounter = 0; // Counter to track loops for NT updates

  public CoralIOTalonFX() {
    // Instantiate the intake motor //
    intakeMotor = new TalonFX(MOTOR_ID);

    // Instantiate the CANrange Flight-of-Time Sensor //
    canRangeNear = new CANrange(CANRANGE_NEAR_CHANNEL);
    canRangeFar = new CANrange(CANRANGE_FAR_CHANNEL);

    // Configure the CANrange Flight-of-Time Sensor //
    canRangeConfigs = new CANrangeConfiguration();
    canRangeConfigs.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    canRangeConfigs.ProximityParams.ProximityHysteresis =
        Units.Meters.of(CORAL_DETECT_CANRANGE_HYSTERESIS).in(Units.Meters);
    canRangeConfigs.ProximityParams.ProximityThreshold =
        Units.Meters.of(CORAL_DETECT_CANRANGE_THRESHOLD).in(Units.Meters);

    canRangeNear.getConfigurator().apply(canRangeConfigs);
    canRangeFar.getConfigurator().apply(canRangeConfigs);

    // Intake Motor Configuration //
    TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(NOMINAL_VOLTAGE)
                .withPeakReverseVoltage(-NOMINAL_VOLTAGE))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(SECONDARY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(STALL_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(FREE_CURRENT_LIMIT)
                .withSupplyCurrentLowerTime(1))
        .withOpenLoopRamps(
            new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(RAMP_RATE_IN_SEC)
                .withTorqueOpenLoopRampPeriod(RAMP_RATE_IN_SEC)
                .withVoltageOpenLoopRampPeriod(RAMP_RATE_IN_SEC))
        .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs())
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    intakeMotor.set(percentage);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    // Only update every 4 loops
    if (ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.setIntakeMotorConnected(intakeMotor.isConnected());

      // Update motor data //
      inputs.setIntakeMotorVoltage(intakeMotor.getMotorVoltage().getValue().in(Units.Volts));
      inputs.setIntakeMotorTemperature(intakeMotor.getDeviceTemp().getValue().in(Units.Fahrenheit));
    }

    ntUpdateCounter++;

    inputs.setIntakeMotorCurrent(intakeMotor.getSupplyCurrent().getValue().in(Units.Amps));

    // Check if the beam break is tripped //
    inputs.setBeamBreakNearTripped(canRangeNear.getIsDetected().getValue());
    inputs.setBeamBreakFarTripped(canRangeFar.getIsDetected().getValue());

    // Check if the current limit is tripped //
    inputs.setIntakeCurrentLimitTripped(
        inputs.getIntakeMotorCurrent() > CORAL_DETECT_CURRENT_THRESHOLD);

    DogLog.log("Coral/IntakeMotor/Connected", inputs.isIntakeMotorConnected());
    DogLog.log("Coral/IntakeMotor/Voltage", inputs.getIntakeMotorVoltage());
    DogLog.log("Coral/IntakeMotor/Current", inputs.getIntakeMotorCurrent());
    DogLog.log("Coral/IntakeMotor/Temperature", inputs.getIntakeMotorTemperature());

    DogLog.log("Coral/CANRange/Near/Tripped", inputs.isBeamBreakNearTripped());
    DogLog.log("Coral/CANRange/Far/Tripped", inputs.isBeamBreakFarTripped());
    DogLog.log("Coral/IntakeMotor/CurrentLimitTripped", inputs.isIntakeCurrentLimitTripped());
  }
}
