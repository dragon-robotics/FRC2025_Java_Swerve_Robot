package frc.robot.subsystems.algae;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class AlgaeIOSparkMax implements AlgaeIO {
  private final SparkMax m_intakeMotor;
  private final SparkMax m_armMotor;

  private final SparkClosedLoopController m_armController;
  private final AbsoluteEncoder m_armAbsEncoder;
  private final ProfiledPIDController m_armPidController;

  // Feedforward for arm motor //
  private final ArmFeedforward m_armFeedforward =
      new ArmFeedforward(ARM_KS, ARM_KG, ARM_KV, ARM_KA);

  private int m_ntUpdateCounter = 0; // Counter to track loops for NT updates

  public AlgaeIOSparkMax() {

    // Instantiate the motors //
    m_intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_armMotor = new SparkMax(ARM_LEAD_MOTOR_ID, MotorType.kBrushless);

    // Instatiate the Sparkmax closed loop controller and the encoder //
    m_armController = m_armMotor.getClosedLoopController();
    m_armAbsEncoder = m_armMotor.getAbsoluteEncoder();

    // Instantiate the armPidController //
    m_armPidController =
        new ProfiledPIDController(
            ARM_P,
            ARM_I,
            ARM_D,
            new Constraints(ARM_MAX_MAXMOTION_VELOCITY, ARM_MAX_MAXMOTION_ACCELERATION));

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
        m_intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Left and Right Arm Motor Configuration //
    SparkMaxConfig m_armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig m_armFollowMotorConfig = new SparkMaxConfig();

    // Left Motor Configuration //
    m_armMotorConfig
        .voltageCompensation(ARM_NOMINAL_VOLTAGE)
        .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake);

    // Left Motor Absolute Encoder Configuration //
    m_armMotorConfig
        .absoluteEncoder
        // .zeroCentered(true)
        .zeroOffset(ABS_ENC_OFFSET_VAL)
        .inverted(true);

    // Left Motor Closed Loop Configuration //
    m_armMotorConfig
        .closedLoop
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

    m_armMotor.configure(
        m_armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Set the motors to start at 0 //
    m_intakeMotor.set(0);
    m_armMotor.set(0);
  }

  private double convertAngleToAbsEncoder(double angle) {
    return angle / (2 * Math.PI);
  }

  private double convertAbsEncoderToAngle(double encoderPosition) {
    return encoderPosition * 2 * Math.PI;
  }

  private double calculateFeedforward(double setpoint) {
    double pidOutput = m_armPidController.calculate(m_armAbsEncoder.getPosition(), setpoint);
    State setpointState = m_armPidController.getSetpoint();
    return m_armFeedforward.calculate(setpointState.position, setpointState.velocity);
  }

  @Override
  public void setArmMotorVoltage(double voltage) {
    m_armMotor.setVoltage(MathUtil.clamp(voltage, -ARM_NOMINAL_VOLTAGE, ARM_NOMINAL_VOLTAGE));
  }

  @Override
  public void setArmMotorPercentage(double percentage) {
    m_armMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setArmSetpoint(double setpoint) {
    m_armController.setReference(setpoint, ControlType.kPosition, PID_SLOT);
  }

  @Override
  public void setArmSetpointFF(double setpoint) {
    m_armController.setReference(
        setpoint,
        ControlType.kMAXMotionPositionControl,
        PID_SLOT,
        calculateFeedforward(setpoint),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    m_intakeMotor.setVoltage(
        MathUtil.clamp(voltage, -INTAKE_NOMINAL_VOLTAGE, INTAKE_NOMINAL_VOLTAGE));
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    m_intakeMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // Only update every 4 loops
    if (m_ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.armMotorConnected = m_armMotor.getDeviceId() == ARM_LEAD_MOTOR_ID;
      inputs.intakeMotorConnected = m_intakeMotor.getDeviceId() == INTAKE_MOTOR_ID;

      // Get arm motor data //
      inputs.armMotorVoltage = m_armMotor.getBusVoltage();
      inputs.armMotorDutyCycle = m_armMotor.getAppliedOutput();
      inputs.armMotorCurrent = m_armMotor.getOutputCurrent();
      inputs.armMotorTemperature = m_armMotor.getMotorTemperature();
      inputs.armMotorPosition = m_armAbsEncoder.getPosition();
      inputs.armMotorVelocity = m_armAbsEncoder.getVelocity();

      // Get intake motor data //
      inputs.intakeMotorVoltage = m_intakeMotor.getBusVoltage();
      inputs.intakeMotorDutyCycle = m_intakeMotor.getAppliedOutput();
      inputs.intakeMotorTemperature = m_intakeMotor.getMotorTemperature();
    }

    m_ntUpdateCounter++;

    inputs.intakeMotorCurrent = m_intakeMotor.getOutputCurrent();

    // Check if the current limit is tripped //
    inputs.intakeCurrentLimitTripped = inputs.intakeMotorCurrent > ALGAE_DETECT_CURRENT_THRESHOLD;

    DogLog.log("Algae/ArmMotor/Connected", inputs.armMotorConnected);
    DogLog.log("Algae/ArmMotor/Voltage", inputs.armMotorVoltage);
    DogLog.log("Algae/ArmMotor/DutyCycle", inputs.armMotorDutyCycle);
    DogLog.log("Algae/ArmMotor/Current", inputs.armMotorCurrent);
    DogLog.log("Algae/ArmMotor/Temperature", inputs.armMotorTemperature);
    DogLog.log("Algae/ArmMotor/Position", inputs.armMotorPosition);
    DogLog.log("Algae/ArmMotor/Velocity", inputs.armMotorVelocity);

    DogLog.log("Algae/IntakeMotor/Connected", inputs.intakeMotorConnected);
    DogLog.log("Algae/IntakeMotor/Voltage", inputs.intakeMotorVoltage);
    DogLog.log("Algae/IntakeMotor/DutyCycle", inputs.intakeMotorDutyCycle);
    DogLog.log("Algae/IntakeMotor/Current", inputs.intakeMotorCurrent);
    DogLog.log("Algae/IntakeMotor/Temperature", inputs.intakeMotorTemperature);
    DogLog.log("Algae/IntakeMotor/CurrentLimitTripped", inputs.intakeCurrentLimitTripped);
  }
}
