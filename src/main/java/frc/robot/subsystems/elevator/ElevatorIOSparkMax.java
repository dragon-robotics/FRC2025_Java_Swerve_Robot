package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorSubsystemConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorLeadMotor = new SparkMax(LEAD_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax elevatorFollowMotor =
      new SparkMax(FOLLOW_MOTOR_ID, MotorType.kBrushless);

  private final SparkClosedLoopController elevatorLeadController =
      elevatorLeadMotor.getClosedLoopController();
  private final SparkMaxAlternateEncoder elevatorLeadRelEncoder =
      (SparkMaxAlternateEncoder) elevatorLeadMotor.getAlternateEncoder();
  private final ProfiledPIDController pidController;

  private int ntUpdateCounter = 0; // Counter to track loops for NT updates

  public ElevatorIOSparkMax() {
    // Lead and Follow Elevator Motor Configuration //
    SparkMaxConfig elevatorLeadMotorConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorFollowMotorConfig = new SparkMaxConfig();

    // Lead Motor Configuration //
    elevatorLeadMotorConfig
        .voltageCompensation(NOMINAL_VOLTAGE)
        .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake);

    // Lead Motor Relative Encoder Configuration //
    elevatorLeadMotorConfig
        .alternateEncoder
        .averageDepth(AVERAGE_DEPTH)
        .countsPerRevolution(COUNTS_PER_REVOLUTION)
        .inverted(false);

    // Lead Motor Closed Loop Configuration //
    elevatorLeadMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(P, I, D, PID_SLOT)
        .outputRange(MIN_OUTPUT, MAX_OUTPUT)
        .maxMotion
        .maxVelocity(MAX_MAXMOTION_VELOCITY)
        .maxAcceleration(MAX_MAXMOTION_ACCELERATION)
        .allowedClosedLoopError(MAXMOTION_ALLOWED_ERROR, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    // Follow Motor Configuration //
    elevatorFollowMotorConfig
        .voltageCompensation(NOMINAL_VOLTAGE)
        .smartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake)
        .follow(LEAD_MOTOR_ID, true);

    elevatorLeadMotor.configure(
        elevatorLeadMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    elevatorFollowMotor.configure(
        elevatorFollowMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the motors to start at 0 //
    elevatorLeadMotor.set(0);
    elevatorFollowMotor.set(0);

    // Set the encoder to be 0 //
    elevatorLeadRelEncoder.setPosition(0);

    // PID Controller
    pidController =
        new ProfiledPIDController(
            P, I, D, new Constraints(MAX_MAXMOTION_VELOCITY, MAX_MAXMOTION_ACCELERATION));

    pidController.setTolerance(MAXMOTION_ALLOWED_ERROR);
  }

  @Override
  public void seedElevatorMotorEncoderPosition(double position) {
    elevatorLeadRelEncoder.setPosition(position);
  }

  @Override
  public void setElevatorMotorVoltage(double voltage) {
    elevatorLeadMotor.setVoltage(MathUtil.clamp(voltage, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
  }

  @Override
  public void setElevatorMotorPercentage(double percentage) {
    elevatorLeadMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setElevatorMotorSetpoint(double setpoint) {
    elevatorLeadController.setReference(setpoint, ControlType.kPosition, PID_SLOT);
  }

  @Override
  public double getElevatorSetpoint() {
    return elevatorLeadRelEncoder.getPosition();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // Only update every 4 loops
    if (ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.setElevatorLeadMotorConnected(elevatorLeadMotor.getDeviceId() == LEAD_MOTOR_ID);
      inputs.setElevatorFollowMotorConnected(elevatorFollowMotor.getDeviceId() == FOLLOW_MOTOR_ID);

      // Update motor data //
      inputs.setElevatorLeadMotorVoltage(elevatorLeadMotor.getBusVoltage());
      inputs.setElevatorLeadMotorDutyCycle(elevatorLeadMotor.getAppliedOutput());
      inputs.setElevatorLeadMotorTemperature(elevatorLeadMotor.getMotorTemperature());
      inputs.setElevatorLeadMotorVelocity(elevatorLeadRelEncoder.getVelocity());

      inputs.setElevatorFollowMotorVoltage(elevatorFollowMotor.getBusVoltage());
      inputs.setElevatorFollowMotorDutyCycle(elevatorFollowMotor.getAppliedOutput());
      inputs.setElevatorFollowMotorCurrent(elevatorFollowMotor.getOutputCurrent());
      inputs.setElevatorFollowMotorTemperature(elevatorFollowMotor.getMotorTemperature());
    }

    ntUpdateCounter++;

    // Check the elevator position //
    inputs.setElevatorLeadMotorPosition(elevatorLeadRelEncoder.getPosition());

    // Check if the current limit is tripped //
    inputs.setElevatorLeadMotorCurrent(elevatorLeadMotor.getOutputCurrent());
    inputs.setElevatorCurrentLimitTripped(inputs.getElevatorLeadMotorCurrent() >= STALL_CURRENT_LIMIT);

    // Check if the elevator is at the slow down threshold //
    inputs.setElevatorAtSlowDownThreshold(inputs.getElevatorLeadMotorPosition() >= HOME);

    DogLog.log("Elevator/LeadMotor/Connected", inputs.isElevatorLeadMotorConnected());
    DogLog.log("Elevator/LeadMotor/Voltage", inputs.getElevatorLeadMotorVoltage());
    DogLog.log("Elevator/LeadMotor/DutyCycle", inputs.getElevatorLeadMotorDutyCycle());
    DogLog.log("Elevator/LeadMotor/Current", inputs.getElevatorLeadMotorCurrent());
    DogLog.log("Elevator/LeadMotor/Temperature", inputs.getElevatorLeadMotorTemperature());
    DogLog.log("Elevator/LeadMotor/Position", inputs.getElevatorLeadMotorPosition());
    DogLog.log("Elevator/LeadMotor/Velocity", inputs.getElevatorLeadMotorVelocity());

    DogLog.log("Elevator/FollowMotor/Connected", inputs.isElevatorFollowMotorConnected());
    DogLog.log("Elevator/FollowMotor/Voltage", inputs.getElevatorFollowMotorVoltage());
    DogLog.log("Elevator/FollowMotor/DutyCycle", inputs.getElevatorFollowMotorDutyCycle());
    DogLog.log("Elevator/FollowMotor/Current", inputs.getElevatorFollowMotorCurrent());
    DogLog.log("Elevator/FollowMotor/Temperature", inputs.getElevatorFollowMotorTemperature());

    DogLog.log("Elevator/CurrentLimitTripped", inputs.isElevatorCurrentLimitTripped());
    DogLog.log("Elevator/AtSlowDownThreshold", inputs.isElevatorAtSlowDownThreshold());
  }
}
