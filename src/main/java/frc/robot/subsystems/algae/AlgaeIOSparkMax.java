package frc.robot.subsystems.algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

public class AlgaeIOSparkMax implements AlgaeIO {
  private final SparkMax m_intakeMotor;
  private final SparkMax m_armLeadMotor;

  private final SparkClosedLoopController m_armLeadController;
  private final AbsoluteEncoder m_armLeadAbsEncoder;
  private final ProfiledPIDController m_armPidController;

  private double currentLimitCheckTime;

  // Feedforward for arm motor //
  private final ArmFeedforward m_armLeadFeedforward = new ArmFeedforward(ARM_KS, ARM_KG, ARM_KV, ARM_KA);

  public AlgaeIOSparkMax() {
    // Initialize current limit check time //
    currentLimitCheckTime = Timer.getFPGATimestamp();

    // Instantiate the motors //
    m_intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_armLeadMotor = new SparkMax(ARM_LEAD_MOTOR_ID, MotorType.kBrushless);

    // Instatiate the Sparkmax closed loop controller and the encoder //
    m_armLeadController = m_armLeadMotor.getClosedLoopController();
    m_armLeadAbsEncoder = m_armLeadMotor.getAbsoluteEncoder();

    // Instantiate the armPidController //
    m_armPidController = new ProfiledPIDController(
        ARM_P,
        ARM_I,
        ARM_D,
        new Constraints(
            ARM_MAX_MAXMOTION_VELOCITY,
            ARM_MAX_MAXMOTION_ACCELERATION));

    m_armPidController.setTolerance(ARM_MAXMOTION_ALLOWED_ERROR);

    // Intake Motor Configuration //
    SparkMaxConfig m_intakeMotorConfig = new SparkMaxConfig();
    m_intakeMotorConfig
      .voltageCompensation(INTAKE_NOMINAL_VOLTAGE)
      .smartCurrentLimit(INTAKE_STALL_CURRENT_LIMIT, INTAKE_FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(INTAKE_STALL_CURRENT_LIMIT)
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

    // Left and Right Arm Motor Configuration //
    SparkMaxConfig m_armLeadMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_armFollowMotorConfig = new SparkMaxConfig();

    // Left Motor Configuration //
    m_armLeadMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // Left Motor Soft Limits //
    // m_armLeadMotorConfig.softLimit
    //   .forwardSoftLimitEnabled(true)
    //   .forwardSoftLimit(0.5)
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(-0.5);

    // Left Motor Absolute Encoder Configuration //
    m_armLeadMotorConfig.absoluteEncoder
      // .zeroCentered(true)
      .zeroOffset(ABS_ENC_OFFSET_VAL)
      .inverted(true);

    // Left Motor Closed Loop Configuration //
    m_armLeadMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pidf(ARM_P, ARM_I, ARM_D, ARM_F, PID_SLOT)
      .minOutput(-0.5, PID_SLOT)
      .maxOutput(0.5, PID_SLOT)
      // .positionWrappingEnabled(true)
      // .positionWrappingInputRange(0.5, -0.5)
      // .positionWrappingMinInput(-0.5)
      // .positionWrappingMaxInput(0.5)
      .outputRange(-1, 1)
      .maxMotion
        .maxVelocity(ARM_MAX_MAXMOTION_VELOCITY)
        .maxAcceleration(ARM_MAX_MAXMOTION_ACCELERATION)
        .allowedClosedLoopError(ARM_MAXMOTION_ALLOWED_ERROR, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    // // Left Motor Signals Configuration //
    // m_armLeadMotorConfig.signals
    //   .absoluteEncoderPositionAlwaysOn(true)       // Turn on absolute encoder position
    //   .absoluteEncoderPositionPeriodMs(5)         // Set absolute encoder position period to 5 ms
    //   .absoluteEncoderVelocityAlwaysOn(true)       // Turn on absolute encoder velocity
    //   .absoluteEncoderVelocityPeriodMs(5)         // Set absolute encoder velocity period to 5 ms
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
    //   .limitsPeriodMs(20)                         // Set limits period to 20 ms
    //   .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
    //   .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
    //   .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
    //   .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
    //   .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
    //   .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
    //   .warningsAlwaysOn(false)                     // Turn off warnings
    //   .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

    // Right Motor Configuration //
    m_armFollowMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake)
      .follow(ARM_LEAD_MOTOR_ID, true);

    // // Right Motor Signals Configuration //
    // m_armFollowMotorConfig.signals
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
    //   .limitsPeriodMs(20)                         // Set limits period to 20 ms
    //   .motorTemperaturePeriodMs(5)                // Set motor temperature period to 5 ms
    //   .outputCurrentPeriodMs(5)                   // Set output current period to 5 ms
    //   .primaryEncoderPositionAlwaysOn(false)       // Turn off primary encoder position
    //   .primaryEncoderPositionPeriodMs(500000)     // Set primary encoder position period to 500000 ms
    //   .primaryEncoderVelocityAlwaysOn(false)       // Turn off primary encoder velocity
    //   .primaryEncoderVelocityPeriodMs(500000)     // Set primary encoder velocity period to 500000 ms
    //   .warningsAlwaysOn(false)                     // Turn off warnings
    //   .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms

    m_armLeadMotor.configure(
        m_armLeadMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the motors to start at 0 //
    m_intakeMotor.set(0);
    m_armLeadMotor.set(0);
  }

  private double convertAngleToAbsEncoder(double angle) {
    return angle / (2 * Math.PI);
  }

  private double convertAbsEncoderToAngle(double encoderPosition) {
    return encoderPosition * 2 * Math.PI;
  }

  private double calculateFeedforward(double setpoint) {
    double pidOutput = m_armPidController.calculate(m_armLeadAbsEncoder.getPosition(), setpoint);
    State setpointState = m_armPidController.getSetpoint();
    return m_armLeadFeedforward.calculate(setpointState.position, setpointState.velocity);
  }

  @Override
  public void setArmMotorVoltage(double voltage) {
    m_armLeadMotor.setVoltage(MathUtil.clamp(voltage, -ARM_NOMINAL_VOLTAGE, ARM_NOMINAL_VOLTAGE));
  }

  @Override
  public void setArmMotorPercentage(double percentage) {
    m_armLeadMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setArmSetpoint(double setpoint) {
    m_armLeadController.setReference(
      setpoint,
      ControlType.kPosition,
      PID_SLOT);
  }

  @Override
  public void setArmSetpointFF(double setpoint) {
    m_armLeadController.setReference(
      setpoint,
      ControlType.kMAXMotionPositionControl,
      PID_SLOT,
      calculateFeedforward(setpoint),
      ArbFFUnits.kVoltage);
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
    inputs.armLeadMotorConnected = m_armLeadMotor.getDeviceId() == ARM_LEAD_MOTOR_ID;
    // inputs.armLeadMotorConnected = m_armFollowMotor.getDeviceId() == ARM_FOLLOW_MOTOR_ID;
    inputs.armLeadMotorConnected = m_intakeMotor.getDeviceId() == INTAKE_MOTOR_ID;

    // Get left arm motor data //
    inputs.armLeadMotorVoltage = m_armLeadMotor.getBusVoltage();
    inputs.armLeadMotorDutyCycle = m_armLeadMotor.getAppliedOutput();
    inputs.armLeadMotorCurrent = m_armLeadMotor.getOutputCurrent();
    inputs.armLeadMotorTemperature = m_armLeadMotor.getMotorTemperature();
    inputs.armLeadMotorPosition = m_armLeadAbsEncoder.getPosition();
    inputs.armLeadMotorVelocity = m_armLeadAbsEncoder.getVelocity();

    // // Get right arm motor data //
    // inputs.armFollowMotorVoltage = m_armFollowMotor.getBusVoltage();
    // inputs.armFollowMotorDutyCycle = m_armLeadMotor.getAppliedOutput();
    // inputs.armFollowMotorCurrent = m_armFollowMotor.getOutputCurrent();
    // inputs.armFollowMotorTemperature = m_armFollowMotor.getMotorTemperature();

    // Get intake motor data //
    inputs.intakeMotorVoltage = m_intakeMotor.getBusVoltage();
    inputs.intakeMotorDutyCycle = m_intakeMotor.getAppliedOutput();
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = inputs.intakeMotorCurrent > ALGAE_DETECT_CURRENT_THRESHOLD;
  }
}
