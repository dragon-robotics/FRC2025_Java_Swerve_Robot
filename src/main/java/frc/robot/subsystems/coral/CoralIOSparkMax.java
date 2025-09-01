package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import static frc.robot.Constants.CoralSubsystemConstants.*;

public class CoralIOSparkMax implements CoralIO {
  private final SparkMax m_intakeMotor;
  private final CANrange m_canRange;
  private final CANrangeConfiguration m_canRangeConfigs;

  private int m_ntUpdateCounter = 0; // Counter to track loops for NT updates

  public CoralIOSparkMax() {

    // Instantiate the intake motor //
    m_intakeMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

    // Instantiate the CANrange Flight-of-Time Sensor //
    m_canRange = new CANrange(0);

    // Configure the CANrange Flight-of-Time Sensor //
    m_canRangeConfigs = new CANrangeConfiguration();
    m_canRangeConfigs.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    m_canRangeConfigs.ProximityParams.ProximityHysteresis = Units.Meters.of(CORAL_DETECT_CANRANGE_HYSTERESIS).in(Units.Meters);
    m_canRangeConfigs.ProximityParams.ProximityThreshold = Units.Meters.of(CORAL_DETECT_CANRANGE_THRESHOLD).in(Units.Meters);
    m_canRange.getConfigurator().apply(m_canRangeConfigs);

    // Intake Motor Configuration //
    SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();
    m_intakeMotorConfig
      .voltageCompensation(NOMINAL_VOLTAGE)
      .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // Apply motor configurations //
    m_intakeMotor.configure(
        m_intakeMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    m_intakeMotor.setVoltage(MathUtil.clamp(voltage, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    m_intakeMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {

    // Only update every 4 loops
    if (m_ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.intakeMotorConnected = m_intakeMotor.getDeviceId() == MOTOR_ID;

      // Update motor data //
      inputs.intakeMotorVoltage = m_intakeMotor.getAppliedOutput();
      inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();
    }
    
    m_ntUpdateCounter++;

    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();

    // Check if the beam break is tripped //
    inputs.beamBreakTripped = m_canRange.getIsDetected().getValue();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = inputs.intakeMotorCurrent > CORAL_DETECT_CURRENT_THRESHOLD;

    DogLog.log("Coral/IntakeMotor/Connected", inputs.intakeMotorConnected);
    DogLog.log("Coral/IntakeMotor/Voltage", inputs.intakeMotorVoltage);
    DogLog.log("Coral/IntakeMotor/Current", inputs.intakeMotorCurrent);
    DogLog.log("Coral/IntakeMotor/Temperature", inputs.intakeMotorTemperature);
    
    DogLog.log("Coral/CANRange/Tripped", inputs.beamBreakTripped);
    DogLog.log("Coral/IntakeMotor/CurrentLimitTripped", inputs.intakeCurrentLimitTripped);
  }
}
