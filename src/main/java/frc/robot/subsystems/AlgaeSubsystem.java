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
// import frc.robot.Constants.AlgaeSubsystemConstants;
import static frc.robot.Constants.AlgaeSubsystemConstants.*;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // @TODO: Create Elastic Tabs for the Arm Subsystem //

  // Arm Motor Controllers //
  private final SparkMax m_armLeft =
      new SparkMax(
          ARM_LEFT_MOTOR_ID,
          MotorType.kBrushless);

  private final SparkMax m_armRight =
      new SparkMax(
          ARM_RIGHT_MOTOR_ID,
          MotorType.kBrushless);
  
  private final SparkMax m_intake =
      new SparkMax(
          INTAKE_MOTOR_ID,
          MotorType.kBrushless);

  // Arm Motor Controller Configurations //
  private final SparkClosedLoopController m_armLeftController = m_armLeft.getClosedLoopController();
  private final SparkAbsoluteEncoder m_armAbsEncoder = m_armLeft.getAbsoluteEncoder();

  // Arm Motor Configurations //
  private final SparkBaseConfig m_armLeftConfig = new SparkMaxConfig();
  private final SparkBaseConfig m_armRightConfig = new SparkMaxConfig();

  // Intake Motor Configurations //
  private final SparkBaseConfig m_intkaeConfig = new SparkMaxConfig();

  // Last setpoint for the arm //
  private double lastSetpoint = INITIAL_GOAL;
  
  public AlgaeSubsystem() {

    // Left Motor Configuration //
    m_armLeftConfig
        .voltageCompensation(ARM_NOMINAL_VOLTAGE)
        .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    // Left Motor Encoder Configuration //
    m_armLeftConfig.absoluteEncoder
        .zeroCentered(true)
        .zeroOffset(ABS_ENC_OFFSET_VAL);

    // Left Motor Soft Limit Configuration //
    m_armLeftConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(0.5)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(-0.5);

    // Left Motor MAXMotion Configuration //
    MAXMotionConfig m_armLeftMaxMotionConfig = new MAXMotionConfig();
    m_armLeftMaxMotionConfig
        .allowedClosedLoopError(0.02, PID_SLOT)
        .maxAcceleration(10, PID_SLOT)
        .maxVelocity(10, PID_SLOT)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, PID_SLOT);

    m_armLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .apply(m_armLeftMaxMotionConfig)
        .p(P, PID_SLOT)
        .i(I, PID_SLOT)
        .d(D, PID_SLOT)
        .velocityFF(F, PID_SLOT)
        .iZone(IZ, PID_SLOT)
        .minOutput(-0.5, PID_SLOT)
        .maxOutput(0.5, PID_SLOT)
        .iMaxAccum(0.0, PID_SLOT)
        .dFilter(0.0, null)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-0.5, 0.5)
        .positionWrappingMinInput(-0.5)
        .positionWrappingMaxInput(0.5);

    // Right Motor Configuration //
    m_armRightConfig
      .voltageCompensation(ARM_NOMINAL_VOLTAGE)
      .smartCurrentLimit(ARM_STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ARM_SECONDARY_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake)
      .follow(ARM_LEFT_MOTOR_ID); // Follow the left motor

    // Right Motor Soft Limit Configuration //
    m_armRightConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(0.5)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(-0.5);

    // Intake motor configurations //
    m_intkaeConfig
        .voltageCompensation(INTAKE_NOMINAL_VOLTAGE)
        .smartCurrentLimit(INTAKE_STALL_CURRENT_LIMIT, INTAKE_FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(INTAKE_SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast);

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the arm motors in test mode //
    } else {
      // m_armLeftConfig.signals
      //     .absoluteEncoderPositionAlwaysOn(true)
      //     .absoluteEncoderPositionPeriodMs(5)
      //     .absoluteEncoderVelocityAlwaysOn(false)
      //     .absoluteEncoderVelocityPeriodMs(500000)
      //     .analogPositionAlwaysOn(false)
      //     .analogPositionPeriodMs(500000)
      //     .analogVelocityAlwaysOn(false)
      //     .analogVelocityPeriodMs(500000)
      //     .analogVoltageAlwaysOn(false)
      //     .analogVoltagePeriodMs(500000)
      //     .appliedOutputPeriodMs(5)
      //     .busVoltagePeriodMs(500000)
      //     .externalOrAltEncoderPosition(500000)
      //     .externalOrAltEncoderPositionAlwaysOn(false)
      //     .externalOrAltEncoderVelocity(500000)
      //     .externalOrAltEncoderVelocityAlwaysOn(false)
      //     .faultsAlwaysOn(false)
      //     .faultsPeriodMs(500000)
      //     .iAccumulationAlwaysOn(false)
      //     .iAccumulationPeriodMs(500000)
      //     .limitsPeriodMs(500000)
      //     .motorTemperaturePeriodMs(500000)
      //     .outputCurrentPeriodMs(5)
      //     .primaryEncoderPositionAlwaysOn(false)
      //     .primaryEncoderPositionPeriodMs(500000)
      //     .primaryEncoderVelocityAlwaysOn(false)
      //     .primaryEncoderVelocityPeriodMs(500000)
      //     .warningsAlwaysOn(false)
      //     .warningsPeriodMs(500000);

      // m_armRightConfig.signals
      //     .absoluteEncoderPositionAlwaysOn(true)
      //     .absoluteEncoderPositionPeriodMs(5)
      //     .absoluteEncoderVelocityAlwaysOn(false)
      //     .absoluteEncoderVelocityPeriodMs(500000)
      //     .analogPositionAlwaysOn(false)
      //     .analogPositionPeriodMs(500000)
      //     .analogVelocityAlwaysOn(false)
      //     .analogVelocityPeriodMs(500000)
      //     .analogVoltageAlwaysOn(false)
      //     .analogVoltagePeriodMs(500000)
      //     .appliedOutputPeriodMs(5)
      //     .busVoltagePeriodMs(500000)
      //     .externalOrAltEncoderPosition(500000)
      //     .externalOrAltEncoderPositionAlwaysOn(false)
      //     .externalOrAltEncoderVelocity(500000)
      //     .externalOrAltEncoderVelocityAlwaysOn(false)
      //     .faultsAlwaysOn(false)
      //     .faultsPeriodMs(500000)
      //     .iAccumulationAlwaysOn(false)
      //     .iAccumulationPeriodMs(500000)
      //     .limitsPeriodMs(500000)
      //     .motorTemperaturePeriodMs(500000)
      //     .outputCurrentPeriodMs(5)
      //     .primaryEncoderPositionAlwaysOn(false)
      //     .primaryEncoderPositionPeriodMs(500000)
      //     .primaryEncoderVelocityAlwaysOn(false)
      //     .primaryEncoderVelocityPeriodMs(500000)
      //     .warningsAlwaysOn(false)
      //     .warningsPeriodMs(500000);
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

    m_intake.configure(
        m_intkaeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the lead and follow arm motor to 0 power //
    m_armLeft.set(0);
    m_armRight.set(0);
    m_intake.set(0);
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
