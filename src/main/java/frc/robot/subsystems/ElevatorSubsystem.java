// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class ElevatorSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Elevator Subsystem //
  
  // Elevator Motor Controllers //
  private final SparkMax m_elevatorLeft = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_elevatorRight = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  
  // Elevator Motor Controller Configurations //
  private final SparkClosedLoopController m_elevatorLeftController = m_elevatorLeft.getClosedLoopController();
  private final SparkMaxAlternateEncoder m_elevatorRelEncoder = (SparkMaxAlternateEncoder) m_elevatorLeft.getAlternateEncoder();

  // Elevator Motor Configurations //
  private final SparkBaseConfig m_elevatorLeftConfig = new SparkMaxConfig();
  private final SparkBaseConfig m_elevatorRightConfig = new SparkMaxConfig();

  private double lastSetpoint = ElevatorConstants.LVL_1;

  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem() {

    // Left Motor Configuration //
    m_elevatorLeftConfig
        .voltageCompensation(ElevatorConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(ElevatorConstants.STALL_CURRENT_LIMIT)
        .secondaryCurrentLimit(ElevatorConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake);

    // Left Motor Encoder Configuration //
    m_elevatorLeftConfig.encoder
        .quadratureAverageDepth(64)
        .quadratureMeasurementPeriod(100);

    // Left Motor Soft Limit Configuration //
    m_elevatorLeftConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(1)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(0);

        // Left Motor MAXMotion Configuration //
    MAXMotionConfig m_elevatorLeftMaxMotionConfig = new MAXMotionConfig();
    m_elevatorLeftMaxMotionConfig
        .allowedClosedLoopError(0.02, ClosedLoopSlot.kSlot0)
        .maxAcceleration(10, ClosedLoopSlot.kSlot0)
        .maxVelocity(10, ClosedLoopSlot.kSlot0)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

    m_elevatorLeftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .apply(m_elevatorLeftMaxMotionConfig)
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

    m_elevatorRightConfig
      .voltageCompensation(ElevatorConstants.NOMINAL_VOLTAGE)
      .smartCurrentLimit(ElevatorConstants.STALL_CURRENT_LIMIT)
      .secondaryCurrentLimit(ElevatorConstants.SECONDARY_CURRENT_LIMIT)
      .idleMode(IdleMode.kBrake)
      .follow(ElevatorConstants.LEFT_MOTOR_ID); // Follow the left motor

    // Right Motor Soft Limit Configuration //
    m_elevatorRightConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(1)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(0);

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the elevator motors in test mode //
    } else {
      m_elevatorLeftConfig.signals
          .absoluteEncoderPositionAlwaysOn(false)
          .absoluteEncoderPositionPeriodMs(500000)
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
          .externalOrAltEncoderPosition(5)
          .externalOrAltEncoderPositionAlwaysOn(true)
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

      m_elevatorRightConfig.signals
          .absoluteEncoderPositionAlwaysOn(false)
          .absoluteEncoderPositionPeriodMs(500000)
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
          .externalOrAltEncoderPosition(5)
          .externalOrAltEncoderPositionAlwaysOn(true)
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
    m_elevatorLeft.configure(
        m_elevatorLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_elevatorRight.configure(
        m_elevatorRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the lead and follow elevator motor to 0 power //
    m_elevatorLeft.set(0);
    m_elevatorRight.set(0);
  }

  /** Set the elevator to a specific speed
   * @param speed the speed to set the elevator to
   */
  public void setElevatorSpeed(double speed) {
    m_elevatorLeft.set(speed);
  }

  /** Set the elevator to a specific voltage
   * @param voltage the voltage to set the elevator to
   */
  public void setElevatorVoltage(double voltage) {
    m_elevatorLeft.setVoltage(voltage);
  }

  /**
   * Set the elevator to climb
   * @param climb true to climb, false to stop
   */
  public void setElevatorClimb(boolean climb) {
    if (climb) {
      m_elevatorLeft.set(1.0);
    } else {
      m_elevatorLeft.set(0);
    }
  }

  /**
   * Set the elevator to descend or descend
   * @param descend true to descend, false to stop
   */
  public void setElevatorDescend(boolean descend) {
    if (descend) {
      m_elevatorLeft.set(-1.0);
    } else {
      m_elevatorLeft.set(0);
    }
  }

  public void setElevatorPosition(double position) {
    m_elevatorLeftController.setReference(position, ControlType.kPosition);
  }

  /**
   * Stop the elevator from moving
   */
  public void stopElevator() {
    m_elevatorLeft.set(0);
    m_elevatorRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Update the Elastic GUI Readouts for the elevator motors when in test mode //
    }
  }
}
