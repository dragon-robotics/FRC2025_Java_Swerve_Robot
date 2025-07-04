package frc.robot.subsystems.algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

  // Feedforward for arm motor //
  private final ArmFeedforward m_armLeadFeedforward = new ArmFeedforward(ARM_KS, ARM_KG, ARM_KV, ARM_KA);

  public AlgaeIOSparkMax() {

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
      .outputRange(-1, 1)
      .maxMotion
        .maxVelocity(ARM_MAX_MAXMOTION_VELOCITY)
        .maxAcceleration(ARM_MAX_MAXMOTION_ACCELERATION)
        .allowedClosedLoopError(ARM_MAXMOTION_ALLOWED_ERROR, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    // Right Motor Configuration //
    m_armFollowMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake)
      .follow(ARM_LEAD_MOTOR_ID, true);

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
    inputs.armLeadMotorConnected = m_intakeMotor.getDeviceId() == INTAKE_MOTOR_ID;

    // Get left arm motor data //
    inputs.armLeadMotorVoltage = m_armLeadMotor.getBusVoltage();
    inputs.armLeadMotorDutyCycle = m_armLeadMotor.getAppliedOutput();
    inputs.armLeadMotorCurrent = m_armLeadMotor.getOutputCurrent();
    inputs.armLeadMotorTemperature = m_armLeadMotor.getMotorTemperature();
    inputs.armLeadMotorPosition = m_armLeadAbsEncoder.getPosition();
    inputs.armLeadMotorVelocity = m_armLeadAbsEncoder.getVelocity();

    // Get intake motor data //
    inputs.intakeMotorVoltage = m_intakeMotor.getBusVoltage();
    inputs.intakeMotorDutyCycle = m_intakeMotor.getAppliedOutput();
    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();
    inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = inputs.intakeMotorCurrent > ALGAE_DETECT_CURRENT_THRESHOLD;
  }
}
