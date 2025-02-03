package frc.robot.subsystems.algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

public class AlgaeIOSparkMax implements AlgaeIO {
  private final SparkMax m_intakeMotor;
  private final SparkMax m_armLeftMotor;
  private final SparkMax m_armRightMotor;

  private final SparkClosedLoopController m_armLeftController;
  private final AbsoluteEncoder m_armLeftAbsEncoder;

  public AlgaeIOSparkMax() {
    m_intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    m_armLeftMotor = new SparkMax(ARM_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_armRightMotor = new SparkMax(ARM_RIGHT_MOTOR_ID, MotorType.kBrushless);

    // Intake Motor Configuration //
    SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();
    m_intakeMotorConfig
      .voltageCompensation(INTAKE_NOMINAL_VOLTAGE)
      .smartCurrentLimit(INTAKE_STALL_CURRENT_LIMIT, INTAKE_FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(INTAKE_STALL_CURRENT_LIMIT)
      .idleMode(IdleMode.kCoast);

    // Intake Motor Signals Configuration //
    m_intakeMotorConfig.signals
      .absoluteEncoderPositionAlwaysOn(false)      // Turn off absolute encoder position
      .absoluteEncoderPositionPeriodMs(500000)    // Set absolute encoder position period to 500000 ms
      .absoluteEncoderVelocityAlwaysOn(false)      // Turn off absolute encoder velocity
      .absoluteEncoderVelocityPeriodMs(500000)    // Set absolute encoder velocity period to 500000 ms
      .analogPositionAlwaysOn(false)               // Turn off analog position
      .analogPositionPeriodMs(500000)             // Set analog position period to 500000 ms
      .analogVelocityAlwaysOn(false)               // Turn off analog velocity
      .analogVelocityPeriodMs(500000)             // Set analog velocity period to 500000 ms
      .analogVoltageAlwaysOn(false)                // Turn off analog voltage
      .analogVoltagePeriodMs(500000)              // Set analog voltage period to 500000 ms
      .appliedOutputPeriodMs(5)                   // Set applied output period to 5 ms
      .busVoltagePeriodMs(5)                      // Set bus voltage period to 5 ms
      .externalOrAltEncoderPositionAlwaysOn(false) // Turn off external or alt encoder position 
      .externalOrAltEncoderPosition(500000)       // Set external or alt encoder position to 500000 ms
      .externalOrAltEncoderVelocityAlwaysOn(false) // Turn off external or alt encoder velocity
      .externalOrAltEncoderVelocity(500000)       // Set external or alt encoder velocity to 500000 ms
      .faultsAlwaysOn(false)                       // Turn off faults
      .faultsPeriodMs(500000)                     // Set faults period to 500000 ms
      .iAccumulationAlwaysOn(false)                // Turn off i accumulation
      .iAccumulationPeriodMs(500000)              // Set i accumulation period to 500000 ms
      .limitsPeriodMs(500000)                     // Set limits period to 500000 ms
      .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
      .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
      .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
      .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
      .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
      .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
      .warningsAlwaysOn(false)                     // Turn off warnings
      .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

    // Apply motor configurations //
    m_intakeMotor.configure(
        m_intakeMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    // Left and Right Arm Motor Configuration //
    SparkMaxConfig m_armLeftMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_armRightMotorConfig = new SparkMaxConfig();

    // Left Motor Configuration //
    m_armLeftMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake);

    // Left Motor Soft Limits //
    m_armLeftMotorConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(0.5)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(-0.5);

    // Left Motor Absolute Encoder Configuration //
    m_armLeftMotorConfig.absoluteEncoder
      .zeroCentered(true)
      .zeroOffset(ABS_ENC_OFFSET_VAL);

    // Left Motor Closed Loop Configuration //
    m_armLeftMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pidf(P, I, D, F, PID_SLOT)
      .minOutput(-0.5, PID_SLOT)
      .maxOutput(0.5, PID_SLOT)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0.5, -0.5)
      .positionWrappingMinInput(-0.5)
      .positionWrappingMaxInput(0.5)
      .outputRange(-1, 1)
      .maxMotion
        .maxVelocity(MAX_MAXMOTION_VELOCITY)
        .maxAcceleration(MAX_MAXMOTION_ACCELERATION)
        .allowedClosedLoopError(MAXMOTION_ALLOWED_ERROR, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    // Left Motor Signals Configuration //
    m_armLeftMotorConfig.signals
      .absoluteEncoderPositionAlwaysOn(true)       // Turn on absolute encoder position
      .absoluteEncoderPositionPeriodMs(5)         // Set absolute encoder position period to 5 ms
      .absoluteEncoderVelocityAlwaysOn(false)      // Turn off absolute encoder velocity
      .absoluteEncoderVelocityPeriodMs(500000)    // Set absolute encoder velocity period to 500000 ms
      .analogPositionAlwaysOn(false)               // Turn off analog position
      .analogPositionPeriodMs(500000)             // Set analog position period to 500000 ms
      .analogVelocityAlwaysOn(false)               // Turn off analog velocity
      .analogVelocityPeriodMs(500000)             // Set analog velocity period to 500000 ms
      .analogVoltageAlwaysOn(false)                // Turn off analog voltage
      .analogVoltagePeriodMs(500000)              // Set analog voltage period to 500000 ms
      .appliedOutputPeriodMs(5)                   // Set applied output period to 5 ms
      .busVoltagePeriodMs(5)                      // Set bus voltage period to 5 ms
      .externalOrAltEncoderPositionAlwaysOn(false) // Turn off external or alt encoder position 
      .externalOrAltEncoderPosition(500000)       // Set external or alt encoder position to 500000 ms
      .externalOrAltEncoderVelocityAlwaysOn(false) // Turn off external or alt encoder velocity
      .externalOrAltEncoderVelocity(500000)       // Set external or alt encoder velocity to 500000 ms
      .faultsAlwaysOn(false)                       // Turn off faults
      .faultsPeriodMs(500000)                     // Set faults period to 500000 ms
      .iAccumulationAlwaysOn(false)                // Turn off i accumulation
      .iAccumulationPeriodMs(500000)              // Set i accumulation period to 500000 ms
      .limitsPeriodMs(20)                         // Set limits period to 20 ms
      .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
      .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
      .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
      .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
      .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
      .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
      .warningsAlwaysOn(false)                     // Turn off warnings
      .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

    // Right Motor Configuration //
    m_armRightMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake)
      .follow(ARM_LEFT_MOTOR_ID);

    // Right Motor Signals Configuration //
    m_armRightMotorConfig.signals
      .absoluteEncoderPositionAlwaysOn(false)      // Turn off absolute encoder position
      .absoluteEncoderPositionPeriodMs(500000)    // Set absolute encoder position period to 500000 ms
      .absoluteEncoderVelocityAlwaysOn(false)      // Turn off absolute encoder velocity
      .absoluteEncoderVelocityPeriodMs(500000)    // Set absolute encoder velocity period to 500000 ms
      .analogPositionAlwaysOn(false)               // Turn off analog position
      .analogPositionPeriodMs(500000)             // Set analog position period to 500000 ms
      .analogVelocityAlwaysOn(false)               // Turn off analog velocity
      .analogVelocityPeriodMs(500000)             // Set analog velocity period to 500000 ms
      .analogVoltageAlwaysOn(false)                // Turn off analog voltage
      .analogVoltagePeriodMs(500000)              // Set analog voltage period to 500000 ms
      .appliedOutputPeriodMs(5)                   // Set applied output period to 5 ms
      .busVoltagePeriodMs(5)                      // Set bus voltage period to 5 ms
      .externalOrAltEncoderPositionAlwaysOn(false) // Turn off external or alt encoder position 
      .externalOrAltEncoderPosition(500000)       // Set external or alt encoder position to 500000 ms
      .externalOrAltEncoderVelocityAlwaysOn(false) // Turn off external or alt encoder velocity
      .externalOrAltEncoderVelocity(500000)       // Set external or alt encoder velocity to 500000 ms
      .faultsAlwaysOn(false)                       // Turn off faults
      .faultsPeriodMs(500000)                     // Set faults period to 500000 ms
      .iAccumulationAlwaysOn(false)                // Turn off i accumulation
      .iAccumulationPeriodMs(500000)              // Set i accumulation period to 500000 ms
      .limitsPeriodMs(20)                         // Set limits period to 20 ms
      .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
      .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
      .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
      .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
      .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
      .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
      .warningsAlwaysOn(false)                     // Turn off warnings
      .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

    m_armLeftMotor.configure(
        m_armLeftMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    m_armRightMotor.configure(
        m_armRightMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    // Closed Loop Controllers and Encoder Instatiations //
    m_armLeftController = m_armLeftMotor.getClosedLoopController();
    m_armLeftAbsEncoder = m_armLeftMotor.getAbsoluteEncoder();

    // Set the motors to start at 0 //
    m_intakeMotor.set(0);
    m_armLeftMotor.set(0);
    m_armRightMotor.set(0);
  }

  @Override
  public void setArmMotorVoltage(double voltage) {
    m_armLeftMotor.setVoltage(MathUtil.clamp(voltage, -ARM_NOMINAL_VOLTAGE, ARM_NOMINAL_VOLTAGE));
  }

  @Override
  public void setArmMotorPercentage(double percentage) {
    m_armLeftMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setArmSetpoint(double setpoint) {
    m_armLeftController.setReference(setpoint, ControlType.kMAXMotionPositionControl, PID_SLOT);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    m_intakeMotor.setVoltage(MathUtil.clamp(voltage, -INTAKE_NOMINAL_VOLTAGE, INTAKE_NOMINAL_VOLTAGE));
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    m_intakeMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // Check if motors are connected //
    inputs.armLeftMotorConnected = m_armLeftMotor.getDeviceId() == ARM_LEFT_MOTOR_ID;
    inputs.armLeftMotorConnected = m_armRightMotor.getDeviceId() == ARM_RIGHT_MOTOR_ID;
    inputs.armLeftMotorConnected = m_intakeMotor.getDeviceId() == INTAKE_MOTOR_ID;

    // Get left arm motor data //
    inputs.armLeftMotorVoltage = m_armLeftMotor.getBusVoltage();
    inputs.armLeftMotorCurrent = m_armLeftMotor.getOutputCurrent();
    inputs.armLeftMotorTemperature = m_armLeftMotor.getMotorTemperature();
    inputs.armLeftMotorSetpoint = m_armLeftAbsEncoder.getPosition();

    // Get right arm motor data //
    inputs.armRightMotorVoltage = m_armRightMotor.getBusVoltage(); 
    inputs.armRightMotorCurrent = m_armRightMotor.getOutputCurrent();
    inputs.armRightMotorTemperature = m_armRightMotor.getMotorTemperature();

    // Get intake motor data //
    inputs.intakeMotorVoltage = m_intakeMotor.getBusVoltage();
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = m_armLeftMotor.getOutputCurrent() > ALGAE_DETECT_CURRENT_THRESHOLD;
  }
}
