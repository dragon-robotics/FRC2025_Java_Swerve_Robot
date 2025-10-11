package frc.robot.subsystems.coral;

import static frc.robot.Constants.CoralSubsystemConstants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

public class CoralIOSparkMax implements CoralIO {
  private final SparkMax intakeMotor;
  private final CANrange canRangeNear; // The CANRange closer to the entry of the end-effector
  private final CANrange canRangeFar; // The CANRange close to the exit of the end-effector
  private final CANrangeConfiguration canRangeConfigs;

  private int ntUpdateCounter = 0; // Counter to track loops for NT updates

  public CoralIOSparkMax() {

    // Instantiate the intake motor //
    intakeMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

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
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .voltageCompensation(NOMINAL_VOLTAGE)
        .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake);

    // Apply motor configurations //
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    intakeMotor.setVoltage(MathUtil.clamp(voltage, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    intakeMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {

    // Only update every 4 loops
    if (ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.setIntakeMotorConnected(intakeMotor.getDeviceId() == MOTOR_ID);

      // Update motor data //
      inputs.setIntakeMotorVoltage(intakeMotor.getAppliedOutput());
      inputs.setIntakeMotorTemperature(intakeMotor.getMotorTemperature());
    }

    ntUpdateCounter++;

    inputs.setIntakeMotorCurrent(intakeMotor.getOutputCurrent());

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
