package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.CoralSubsystemConstants.*;

public class CoralIOSparkMax implements CoralIO {
  private final SparkMax m_intakeMotor;
  private final CANrange m_canRange;
  private final CANrangeConfiguration m_canRangeConfigs;

  public CoralIOSparkMax() {

    // Instantiate the intake motor //
    m_intakeMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);

    // Instantiate the CANdi Beam Break Sensor //
    m_canRange = new CANrange(0);

    // Configure the CANdi Beam Break Sensor //
    m_canRangeConfigs = new CANrangeConfiguration();
    m_canRangeConfigs.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    m_canRangeConfigs.ProximityParams.ProximityThreshold = Units.Meters.of(0.1).in(Units.Meters);
    m_canRange.getConfigurator().apply(m_canRangeConfigs);

    // Intake Motor Configuration //
    SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();
    m_intakeMotorConfig
      .voltageCompensation(NOMINAL_VOLTAGE)
      .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // // Intake Motor Signals Configuration //
    // m_intakeMotorConfig.signals
    //   .absoluteEncoderPositionAlwaysOn(false)      // Turn off absolute encoder position
    //   .absoluteEncoderPositionPeriodMs(500000)    // Set absolute encoder position period to 500000 ms
    //   .absoluteEncoderVelocityAlwaysOn(false)      // Turn off absolute encoder velocity
    //   .absoluteEncoderVelocityPeriodMs(500000)    // Set absolute encoder velocity period to 500000 ms
    //   .analogPositionAlwaysOn(false)               // Turn off analog position
    //   .analogPositionPeriodMs(500000)             // Set analog position period to 500000 ms
    //   .analogVelocityAlwaysOn(false)               // Turn off analog velocity
    //   .analogVelocityPeriodMs(500000)             // Set analog velocity period to 500000 ms
    //   .analogVoltageAlwaysOn(false)                // Turn off analog voltage
    //   .analogVoltagePeriodMs(500000)              // Set analog voltage period to 500000 ms
    //   .appliedOutputPeriodMs(5)                   // Set applied output period to 5 ms
    //   .busVoltagePeriodMs(5)                      // Set bus voltage period to 5 ms
    //   .externalOrAltEncoderPositionAlwaysOn(false) // Turn off external or alt encoder position 
    //   .externalOrAltEncoderPosition(500000)       // Set external or alt encoder position to 500000 ms
    //   .externalOrAltEncoderVelocityAlwaysOn(false) // Turn off external or alt encoder velocity
    //   .externalOrAltEncoderVelocity(500000)       // Set external or alt encoder velocity to 500000 ms
    //   .faultsAlwaysOn(false)                       // Turn off faults
    //   .faultsPeriodMs(500000)                     // Set faults period to 500000 ms
    //   .iAccumulationAlwaysOn(false)                // Turn off i accumulation
    //   .iAccumulationPeriodMs(500000)              // Set i accumulation period to 500000 ms
    //   .limitsPeriodMs(500000)                     // Set limits period to 500000 ms
    //   .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
    //   .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
    //   .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
    //   .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
    //   .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
    //   .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
    //   .warningsAlwaysOn(false)                     // Turn off warnings
    //   .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

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
    // Check if motors are connected //
    inputs.intakeMotorConnected = m_intakeMotor.getDeviceId() == MOTOR_ID;

    // Update motor data //
    inputs.intakeMotorVoltage = m_intakeMotor.getAppliedOutput();
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();

    // Check if the beam break is tripped //
    inputs.beamBreakTripped = m_canRange.getIsDetected().getValue();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = m_intakeMotor.getOutputCurrent() > CORAL_DETECT_CURRENT_THRESHOLD;
  }
}
