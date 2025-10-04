package frc.robot.subsystems.algae;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AlgaeIOSparkMax implements AlgaeIO {
  private final SparkMax intakeMotor;
  private final SparkMax armMotor;

  private final SparkClosedLoopController armController;
  private final AbsoluteEncoder armAbsEncoder;
  private final ProfiledPIDController armPidController;

  private int ntUpdateCounter = 0; // Counter to track loops for NT updates

  public AlgaeIOSparkMax() {

    // Instantiate the motors //
    intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    armMotor = new SparkMax(ARM_LEAD_MOTOR_ID, MotorType.kBrushless);

    // Instatiate the Sparkmax closed loop controller and the encoder //
    armController = armMotor.getClosedLoopController();
    armAbsEncoder = armMotor.getAbsoluteEncoder();

    // Instantiate the armPidController //
    armPidController =
        new ProfiledPIDController(
            ARM_P,
            ARM_I,
            ARM_D,
            new Constraints(ARM_MAX_MAXMOTION_VELOCITY, ARM_MAX_MAXMOTION_ACCELERATION));

    armPidController.setTolerance(ARM_MAXMOTION_ALLOWED_ERROR);

    // Intake Motor Configuration //
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .voltageCompensation(INTAKE_NOMINAL_VOLTAGE)
        .smartCurrentLimit(INTAKE_STALL_CURRENT_LIMIT, INTAKE_FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(INTAKE_STALL_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    // Apply motor configurations //
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Left and Right Arm Motor Configuration //
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig armFollowMotorConfig = new SparkMaxConfig();

    // Left Motor Configuration //
    armMotorConfig
        .voltageCompensation(ARM_NOMINAL_VOLTAGE)
        .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake);

    // Left Motor Absolute Encoder Configuration //
    armMotorConfig
        .absoluteEncoder
        // .zeroCentered(true)
        .zeroOffset(ABS_ENC_OFFSET_VAL)
        .inverted(true);

    // Left Motor Closed Loop Configuration //
    armMotorConfig
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
    armFollowMotorConfig
        .voltageCompensation(ARM_NOMINAL_VOLTAGE)
        .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
        .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
        .idleMode(IdleMode.kBrake)
        .follow(ARM_LEAD_MOTOR_ID, true);

    armMotor.configure(
        armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Set the motors to start at 0 //
    intakeMotor.set(0);
    armMotor.set(0);
  }

  @Override
  public void setArmMotorVoltage(double voltage) {
    armMotor.setVoltage(MathUtil.clamp(voltage, -ARM_NOMINAL_VOLTAGE, ARM_NOMINAL_VOLTAGE));
  }

  @Override
  public void setArmMotorPercentage(double percentage) {
    armMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void setArmSetpoint(double setpoint) {
    armController.setReference(setpoint, ControlType.kPosition, PID_SLOT);
  }

  @Override
  public void setIntakeMotorVoltage(double voltage) {
    intakeMotor.setVoltage(
        MathUtil.clamp(voltage, -INTAKE_NOMINAL_VOLTAGE, INTAKE_NOMINAL_VOLTAGE));
  }

  @Override
  public void setIntakeMotorPercentage(double percentage) {
    intakeMotor.set(MathUtil.clamp(percentage, -1, 1));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {

    // Only update every 4 loops
    if (ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.setArmMotorConnected(armMotor.getDeviceId() == ARM_LEAD_MOTOR_ID);
      inputs.setIntakeMotorConnected(intakeMotor.getDeviceId() == INTAKE_MOTOR_ID);

      // Get arm motor data //
      inputs.setArmMotorVoltage(armMotor.getBusVoltage());
      inputs.setArmMotorDutyCycle(armMotor.getAppliedOutput());
      inputs.setArmMotorCurrent(armMotor.getOutputCurrent());
      inputs.setArmMotorTemperature(armMotor.getMotorTemperature());
      inputs.setArmMotorPosition(armAbsEncoder.getPosition());
      inputs.setArmMotorVelocity(armAbsEncoder.getVelocity());

      // Get intake motor data //
      inputs.setIntakeMotorVoltage(intakeMotor.getBusVoltage());
      inputs.setIntakeMotorDutyCycle(intakeMotor.getAppliedOutput());
      inputs.setIntakeMotorTemperature(intakeMotor.getMotorTemperature());
    }

    ntUpdateCounter++;

    inputs.setIntakeMotorCurrent(intakeMotor.getOutputCurrent());

    // Check if the current limit is tripped //
    inputs.setIntakeCurrentLimitTripped(
        inputs.getIntakeMotorCurrent() > ALGAE_DETECT_CURRENT_THRESHOLD);

    DogLog.log("Algae/ArmMotor/Connected", inputs.isArmMotorConnected());
    DogLog.log("Algae/ArmMotor/Voltage", inputs.getArmMotorVoltage());
    DogLog.log("Algae/ArmMotor/DutyCycle", inputs.getArmMotorDutyCycle());
    DogLog.log("Algae/ArmMotor/Current", inputs.getArmMotorCurrent());
    DogLog.log("Algae/ArmMotor/Temperature", inputs.getArmMotorTemperature());
    DogLog.log("Algae/ArmMotor/Position", inputs.getArmMotorPosition());
    DogLog.log("Algae/ArmMotor/Velocity", inputs.getArmMotorVelocity());

    DogLog.log("Algae/IntakeMotor/Connected", inputs.isIntakeMotorConnected());
    DogLog.log("Algae/IntakeMotor/Voltage", inputs.getIntakeMotorVoltage());
    DogLog.log("Algae/IntakeMotor/DutyCycle", inputs.getIntakeMotorDutyCycle());
    DogLog.log("Algae/IntakeMotor/Current", inputs.getIntakeMotorCurrent());
    DogLog.log("Algae/IntakeMotor/Temperature", inputs.getIntakeMotorTemperature());
    DogLog.log("Algae/IntakeMotor/CurrentLimitTripped", inputs.isIntakeCurrentLimitTripped());
  }
}
