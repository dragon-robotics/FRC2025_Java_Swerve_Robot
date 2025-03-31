package frc.robot.subsystems.boomstick;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.AlgaeSubsystemConstants.ALGAE_DETECT_CURRENT_THRESHOLD;
import static frc.robot.Constants.AlgaeSubsystemConstants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.BoomstickSubsystemConstants.*;

public class BoomstickIOSparkMax implements BoomstickIO {
  private final SparkMax m_armMotor;

  private final SparkClosedLoopController m_armController;
  private final AbsoluteEncoder m_armAbsEncoder;

  public BoomstickIOSparkMax() {
    // Instantiate the Motors //
    m_armMotor = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);

    // Instatiate the Sparkmax closed loop controller and the encoder //
    m_armController = m_armMotor.getClosedLoopController();
    m_armAbsEncoder = m_armMotor.getAbsoluteEncoder();

    // Boomstick Arm Motor Configuration //
    SparkMaxConfig m_armMotorConfig = new SparkMaxConfig();

    // Left Motor Configuration //
    m_armMotorConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .openLoopRampRate(ARM_RAMP_RATE_IN_SEC)
      .idleMode(IdleMode.kBrake);

    // Left Motor Absolute Encoder Configuration //
    m_armMotorConfig.absoluteEncoder
      // .zeroCentered(true)
      .zeroOffset(ABS_ENC_OFFSET_VAL)
      .inverted(true);

    // Left Motor Closed Loop Configuration //
    m_armMotorConfig.closedLoop
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

    m_armMotor.configure(
        m_armMotorConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters); 
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
    m_armController.setReference(
      setpoint,
      ControlType.kPosition,
      PID_SLOT);
  }

  @Override
  public void updateInputs(BoomstickIOInputs inputs) {

    // Check if motors are connected //
    inputs.armMotorConnected = m_armMotor.getDeviceId() == ARM_MOTOR_ID;

    // Get left arm motor data //
    inputs.armMotorVoltage = m_armMotor.getBusVoltage();
    inputs.armMotorDutyCycle = m_armMotor.getAppliedOutput();
    inputs.armMotorCurrent = m_armMotor.getOutputCurrent();
    inputs.armMotorTemperature = m_armMotor.getMotorTemperature();
    inputs.armMotorPosition = m_armAbsEncoder.getPosition();
    inputs.armMotorVelocity = m_armAbsEncoder.getVelocity();
  }  
}
