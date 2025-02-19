package frc.robot.subsystems.elevator;

import static frc.robot.Constants.AlgaeSubsystemConstants.INTAKE_MOTOR_ID;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import static frc.robot.Constants.ElevatorSubsystemConstants.*;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax m_elevatorLeadMotor = new SparkMax(LEAD_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_elevatorFollowMotor = new SparkMax(FOLLOW_MOTOR_ID, MotorType.kBrushless);

  private final SparkClosedLoopController m_elevatorLeadController = m_elevatorLeadMotor.getClosedLoopController();
  private final SparkMaxAlternateEncoder m_elevatorLeadRelEncoder = (SparkMaxAlternateEncoder) m_elevatorLeadMotor.getAlternateEncoder();
  private final ProfiledPIDController m_pidController;

  // Feedforward for elevator motor //
  private final ElevatorFeedforward m_elevatorLeadFeedforward = new ElevatorFeedforward(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

  public ElevatorIOSparkMax() {
    // Lead and Follow Elevator Motor Configuration //
    SparkMaxConfig m_elevatorLeadMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_elevatorFollowMotorConfig = new SparkMaxConfig();

    // Lead Motor Configuration //
    m_elevatorLeadMotorConfig
      .voltageCompensation(NOMINAL_VOLTAGE)
      .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // Lead Motor Soft Limits //
    m_elevatorLeadMotorConfig.softLimit
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(ENDING_LIMIT)
      .reverseSoftLimitEnabled(true)
      .reverseSoftLimit(STARTING_LIMIT);

    // Lead Motor Relative Encoder Configuration //
    m_elevatorLeadMotorConfig.alternateEncoder
      .averageDepth(AVERAGE_DEPTH)
      .countsPerRevolution(COUNTS_PER_REVOLUTION)
      .inverted(false);

    // Lead Motor Closed Loop Configuration //
    m_elevatorLeadMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
      .pid(P, I, D, PID_SLOT)
      .outputRange(MIN_OUTPUT, MAX_OUTPUT)
      .maxMotion
        .maxVelocity(MAX_MAXMOTION_VELOCITY)
        .maxAcceleration(MAX_MAXMOTION_ACCELERATION)
        .allowedClosedLoopError(MAXMOTION_ALLOWED_ERROR, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    // Lead Motor Signals Configuration //
    m_elevatorLeadMotorConfig.signals
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
      .externalOrAltEncoderPositionAlwaysOn(true)  // Turn on external or alt encoder position
      .externalOrAltEncoderPosition(5)            // Set external or alt encoder position period to 5 ms
      .externalOrAltEncoderVelocityAlwaysOn(true)  // Turn on external or alt encoder velocity
      .externalOrAltEncoderVelocity(5)            // Set external or alt encoder velocity period to 5 ms
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
      .warningsPeriodMs(500000);                  // Set warnings period to 500000 ms                 // Set warnings period to 500000 ms

    // Follow Motor Configuration //
    m_elevatorFollowMotorConfig
      .voltageCompensation(NOMINAL_VOLTAGE)
      .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
      .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // Follow Motor Signal Conifguration //
    m_elevatorFollowMotorConfig.signals
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

    m_elevatorLeadMotor.configure(
        m_elevatorLeadMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    m_elevatorFollowMotor.configure(
        m_elevatorFollowMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the motors to start at 0 //
    m_elevatorLeadMotor.set(0);
    m_elevatorFollowMotor.set(0);

    // Set the encoder to be 0 //
    m_elevatorLeadRelEncoder.setPosition(0);

    // PID Controller
    m_pidController = new ProfiledPIDController(
        P,
        I,
        D,
        new Constraints(
            MAX_MAXMOTION_VELOCITY,
            MAX_MAXMOTION_ACCELERATION));

    m_pidController.setTolerance(MAXMOTION_ALLOWED_ERROR);
  }

  private double calculateFeedforward(double setpoint) {
    double pidOutput = m_pidController.calculate(m_elevatorLeadRelEncoder.getPosition(), setpoint);
    State setpointState = m_pidController.getSetpoint();
    return m_elevatorLeadFeedforward.calculate(setpointState.velocity);
  }

  @Override
  public void seedElevatorMotorEncoderPosition(double position) {
    m_elevatorLeadRelEncoder.setPosition(position);
  }

  @Override
  public void setElevatorMotorVoltage(double voltage) {
    m_elevatorLeadMotor.setVoltage(MathUtil.clamp(voltage, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
  }

  @Override
  public void setElevatorMotorPercentage(double percentage) {
    m_elevatorLeadMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setElevatorMotorSetpoint(double setpoint) {
    m_elevatorLeadController.setReference(
      setpoint,
      ControlType.kMAXMotionPositionControl,
      PID_SLOT);
  }

  @Override
  public void setElevatorSetpointFF(double setpoint) {
    m_elevatorLeadController.setReference(
      setpoint,
      ControlType.kMAXMotionPositionControl,
      PID_SLOT,
      calculateFeedforward(setpoint),
      ArbFFUnits.kVoltage);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Check if motors are connected //
    inputs.elevatorLeadMotorConnected = m_elevatorLeadMotor.getDeviceId() == LEAD_MOTOR_ID;
    inputs.elevatorFollowMotorConnected = m_elevatorFollowMotor.getDeviceId() == FOLLOW_MOTOR_ID;

    // Update motor data //
    inputs.elevatorLeadMotorVoltage = m_elevatorLeadMotor.getBusVoltage();
    inputs.elevatorLeadMotorDutyCycle = m_elevatorLeadMotor.getAppliedOutput();
    inputs.elevatorLeadMotorCurrent = m_elevatorLeadMotor.getOutputCurrent();
    inputs.elevatorLeadMotorTemperature = m_elevatorLeadMotor.getMotorTemperature();
    inputs.elevatorLeadMotorPosition = m_elevatorLeadMotor.getAlternateEncoder().getPosition();
    inputs.elevatorLeadMotorVelocity = m_elevatorLeadMotor.getAlternateEncoder().getVelocity();

    inputs.elevatorFollowMotorVoltage = m_elevatorFollowMotor.getBusVoltage();
    inputs.elevatorFollowMotorDutyCycle = m_elevatorFollowMotor.getAppliedOutput();
    inputs.elevatorFollowMotorCurrent = m_elevatorFollowMotor.getOutputCurrent();
    inputs.elevatorFollowMotorTemperature = m_elevatorFollowMotor.getMotorTemperature();

    // Check if the current limit is tripped //
    inputs.elevatorCurrentLimitTripped = m_elevatorLeadMotor.getOutputCurrent() >= STALL_CURRENT_LIMIT;

    // Check if the elevator is at the slow down threshold //
    inputs.elevatorAtSlowDownThreshold = m_elevatorLeadMotor.getAlternateEncoder().getPosition() >= HOME_GOAL;
  }
}
