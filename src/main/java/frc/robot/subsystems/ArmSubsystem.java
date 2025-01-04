// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // @TODO: Create Elastic Tabs for the Arm Subsystem //

  // Arm Motor Controllers //
  private final SparkMax m_armLeft = new SparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_armRight = new SparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  // Arm Motor Controller Configurations //
  private final SparkClosedLoopController m_armLeftController = m_armLeft.getClosedLoopController();
  private final SparkAbsoluteEncoder m_armAbsEncoder = m_armLeft.getAbsoluteEncoder();

  // Arm Motor Configurations //
  private final SparkBaseConfig m_armLeftConfig = new SparkMaxConfig();
  private final SparkBaseConfig m_armRightConfig = new SparkMaxConfig();

  private double lastSetpoint = ArmConstants.INITIAL_GOAL;
  
  public ArmSubsystem() {

    // Left Motor Configuration //
    m_armLeftConfig
        .voltageCompensation(ArmConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    // Left Motor Encoder Configuration //
    m_armLeftConfig.absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(ArmConstants.ABS_ENC_OFFSET_VAL);

    // Left Motor Soft Limit Configuration //
    m_armLeftConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(0.5)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(-0.5);

    // Left Motor MAXMotion Configuration //
    MAXMotionConfig m_armLeftMaxMotionConfig = new MAXMotionConfig();
    m_armLeftMaxMotionConfig
        .allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0)
        .maxAcceleration(10, ClosedLoopSlot.kSlot0)
        .maxVelocity(10, ClosedLoopSlot.kSlot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

    m_armLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .apply(m_armLeftMaxMotionConfig)
        .p(ArmConstants.P, ClosedLoopSlot.kSlot0)
        .i(ArmConstants.I, ClosedLoopSlot.kSlot0)
        .d(ArmConstants.D, ClosedLoopSlot.kSlot0)
        .velocityFF(ArmConstants.F, ClosedLoopSlot.kSlot0)
        .iZone(ArmConstants.IZ, ClosedLoopSlot.kSlot0)
        .minOutput(-0.5, ClosedLoopSlot.kSlot0)
        .maxOutput(0.5, ClosedLoopSlot.kSlot0)
        .iMaxAccum(0.0, ClosedLoopSlot.kSlot0)
        .dFilter(0.0, null)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-0.5, 0.5)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);

    // Right Motor Configuration //

    m_armRightConfig
      .voltageCompensation(ArmConstants.NOMINAL_VOLTAGE)
      .smartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake)
      .follow(ArmConstants.LEFT_MOTOR_ID); // Follow the left motor

    // Right Motor Soft Limit Configuration //
    m_armRightConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(0.5)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(-0.5);

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the arm motors in test mode //
    } else {
      m_armLeftConfig.signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs(5)
          .absoluteEncoderVelocityAlwaysOn(false)
          .absoluteEncoderVelocityPeriodMs(500000)
          .analogPositionAlwaysOn(false)
          .analogPositionPeriodMs(500000)
          .analogVelocityAlwaysOn(false)
          .analogVelocityPeriodMs(500000)
          .analogVoltageAlwaysOn(false)
          .analogVoltagePeriodMs(500000)
          .appliedOutputPeriodMs(5)
          .busVoltagePeriodMs(500000)
          .externalOrAltEncoderPosition(500000)
          .externalOrAltEncoderPositionAlwaysOn(false)
          .externalOrAltEncoderVelocity(500000)
          .externalOrAltEncoderVelocityAlwaysOn(false)
          .faultsAlwaysOn(false)
          .faultsPeriodMs(500000)
          .iAccumulationAlwaysOn(false)
          .iAccumulationPeriodMs(500000)
          .limitsPeriodMs(500000)
          .motorTemperaturePeriodMs(500000)
          .outputCurrentPeriodMs(5)
          .primaryEncoderPositionAlwaysOn(false)
          .primaryEncoderPositionPeriodMs(500000)
          .primaryEncoderVelocityAlwaysOn(false)
          .primaryEncoderVelocityPeriodMs(500000)
          .warningsAlwaysOn(false)
          .warningsPeriodMs(500000);

      m_armRightConfig.signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs(5)
          .absoluteEncoderVelocityAlwaysOn(false)
          .absoluteEncoderVelocityPeriodMs(500000)
          .analogPositionAlwaysOn(false)
          .analogPositionPeriodMs(500000)
          .analogVelocityAlwaysOn(false)
          .analogVelocityPeriodMs(500000)
          .analogVoltageAlwaysOn(false)
          .analogVoltagePeriodMs(500000)
          .appliedOutputPeriodMs(5)
          .busVoltagePeriodMs(500000)
          .externalOrAltEncoderPosition(500000)
          .externalOrAltEncoderPositionAlwaysOn(false)
          .externalOrAltEncoderVelocity(500000)
          .externalOrAltEncoderVelocityAlwaysOn(false)
          .faultsAlwaysOn(false)
          .faultsPeriodMs(500000)
          .iAccumulationAlwaysOn(false)
          .iAccumulationPeriodMs(500000)
          .limitsPeriodMs(500000)
          .motorTemperaturePeriodMs(500000)
          .outputCurrentPeriodMs(5)
          .primaryEncoderPositionAlwaysOn(false)
          .primaryEncoderPositionPeriodMs(500000)
          .primaryEncoderVelocityAlwaysOn(false)
          .primaryEncoderVelocityPeriodMs(500000)
          .warningsAlwaysOn(false)
          .warningsPeriodMs(500000);
    }

    // Finalize Configuration //
    m_armLeft.configure(
        m_armLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_armRight.configure(
        m_armRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the lead and follow arm motor to 0 power //
    m_armLeft.set(0);
    m_armRight.set(0);
  }

  //#region Arm Test Methods //
  
  public void setArmSpeed(double speed) {
    m_armLeft.set(speed);
  }

  public void setArmPosition(double position) {
    m_armLeftController.setReference(position, ControlType.kPosition);
  }

  public void setArmVoltage(double voltage) {
    if (voltage < -2) {
      voltage = -2;
    } else if (voltage > 2) {
      voltage = 2;
    }
    m_armLeft.setVoltage(voltage);
  }

  public void stopArm() {
    m_armLeft.set(0);
  }

  //#endregion

  /**
   * Get the current position of the amp
   * @return
   */
  public double getArmPosition() {
    // System.out.println("Arm Position: " + m_armAbsEncoder.getPosition());
    return m_armAbsEncoder.getPosition();
  }

  public void setLastSetpoint(double setpoint) {
    lastSetpoint = setpoint;
  }

  public double getLastSetpoint() {
    return lastSetpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Update the Elastic GUI Readouts for the arm motors when in test mode //
    }
  }
}
