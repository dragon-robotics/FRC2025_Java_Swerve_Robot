package frc.robot.subsystems.boomstick;

import static frc.robot.Constants.BoomstickSubsystemConstants.*;

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

public class BoomstickIOSparkMax implements BoomstickIO {
  private final SparkMax armMotor;

  private final SparkClosedLoopController armController;
  private final AbsoluteEncoder armAbsEncoder;

  private int ntUpdateCounter = 0; // Counter to track loops for NT updates

  public BoomstickIOSparkMax() {
    // Instantiate the Motors //
    armMotor = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);

    // Instatiate the Sparkmax closed loop controller and the encoder //
    armController = armMotor.getClosedLoopController();
    armAbsEncoder = armMotor.getAbsoluteEncoder();

    // Boomstick Arm Motor Configuration //
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();

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

    armMotor.configure(
        armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
  public void updateInputs(BoomstickIOInputs inputs) {

    // Only update every 4 loops
    if (ntUpdateCounter % 4 == 0) {
      // Check if motors are connected //
      inputs.setArmMotorConnected(armMotor.getDeviceId() == ARM_MOTOR_ID);

      // Get arm motor data //
      inputs.setArmMotorVoltage(armMotor.getBusVoltage());
      inputs.setArmMotorDutyCycle(armMotor.getAppliedOutput());
      inputs.setArmMotorCurrent(armMotor.getOutputCurrent());
      inputs.setArmMotorTemperature(armMotor.getMotorTemperature());
      inputs.setArmMotorPosition(armAbsEncoder.getPosition());
      inputs.setArmMotorVelocity(armAbsEncoder.getVelocity());
    }

    ntUpdateCounter++;

    DogLog.log("Boomstick/ArmMotor/Connected", inputs.isArmMotorConnected());
    DogLog.log("Boomstick/ArmMotor/Voltage", inputs.getArmMotorVoltage());
    DogLog.log("Boomstick/ArmMotor/DutyCycle", inputs.getArmMotorDutyCycle());
    DogLog.log("Boomstick/ArmMotor/Current", inputs.getArmMotorCurrent());
    DogLog.log("Boomstick/ArmMotor/Temperature", inputs.getArmMotorTemperature());
    DogLog.log("Boomstick/ArmMotor/Position", inputs.getArmMotorPosition());
    DogLog.log("Boomstick/ArmMotor/Velocity", inputs.getArmMotorVelocity());
  }
}
