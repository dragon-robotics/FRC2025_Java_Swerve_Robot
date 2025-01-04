// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Shooter Subsystem //

  // Shooter Motor Controllers //
  private final SparkMax m_shooterTop = new SparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_shooterBottom = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  
  // Shooter Motor Controller Configurations //
  // private final SparkClosedLoopController m_shooterTopController = m_shooterTop.getClosedLoopController();

  // Shooter Motor Configurations //
  private final SparkBaseConfig m_shooterTopConfig = new SparkMaxConfig();
  private final SparkBaseConfig m_shooterBottomConfig = new SparkMaxConfig();

  /** Creates a new ShooterSmartVelocitySubsystem. */
  public ShooterSubsystem() {

    // Top Motor Configuration //
    m_shooterTopConfig
        .voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(ShooterConstants.RAMP_RATE_IN_SEC);

    // Bottom Motor Configuration //
    m_shooterBottomConfig
        .voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(ShooterConstants.RAMP_RATE_IN_SEC)
        .follow(ShooterConstants.TOP_MOTOR_ID);

    // Finalize Configuration //
    m_shooterTop.configure(
        m_shooterTopConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_shooterBottom.configure(
        m_shooterBottomConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the top shooter to 0 RPM //
    m_shooterTop.set(0);
    m_shooterBottom.set(0);

    // Create Shuffleboard entries for the ShooterSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the shooter motors in test mode //
    } else {
      m_shooterTopConfig.signals
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
          .busVoltagePeriodMs(20)
          .externalOrAltEncoderPosition(500000)
          .externalOrAltEncoderPositionAlwaysOn(false)
          .externalOrAltEncoderVelocity(500000)
          .externalOrAltEncoderVelocityAlwaysOn(false)
          .faultsAlwaysOn(false)
          .faultsPeriodMs(10)
          .iAccumulationAlwaysOn(false)
          .iAccumulationPeriodMs(500000)
          .limitsPeriodMs(500000)
          .motorTemperaturePeriodMs(20)
          .outputCurrentPeriodMs(20)
          .primaryEncoderPositionAlwaysOn(false)
          .primaryEncoderPositionPeriodMs(500000)
          .primaryEncoderVelocityAlwaysOn(false)
          .primaryEncoderVelocityPeriodMs(20)
          .warningsAlwaysOn(false)
          .warningsPeriodMs(500000);

      m_shooterBottomConfig.signals
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
          .busVoltagePeriodMs(20)
          .externalOrAltEncoderPosition(500000)
          .externalOrAltEncoderPositionAlwaysOn(false)
          .externalOrAltEncoderVelocity(500000)
          .externalOrAltEncoderVelocityAlwaysOn(false)
          .faultsAlwaysOn(false)
          .faultsPeriodMs(10)
          .iAccumulationAlwaysOn(false)
          .iAccumulationPeriodMs(500000)
          .limitsPeriodMs(500000)
          .motorTemperaturePeriodMs(20)
          .outputCurrentPeriodMs(20)
          .primaryEncoderPositionAlwaysOn(false)
          .primaryEncoderPositionPeriodMs(500000)
          .primaryEncoderVelocityAlwaysOn(false)
          .primaryEncoderVelocityPeriodMs(20)
          .warningsAlwaysOn(false)
          .warningsPeriodMs(500000);
    }
  }

  public void set(double speed) {
    m_shooterTop.set(speed);
  }

  public void setVoltage(double voltage) {
    m_shooterTop.setVoltage(voltage);
  }

  /**
   * Set the shooter percent output
   * @param percentOutput
   */
  public void setShooterPercentOutput(double percentOutput) {
    m_shooterTop.set(percentOutput);
  }

  public void setIdleMode(IdleMode mode) {
    m_shooterTopConfig.idleMode(mode);
    m_shooterBottomConfig.idleMode(mode);

    // Finalize Configuration //
    m_shooterTop.configure(
        m_shooterTopConfig,
        ResetMode.kNoResetSafeParameters, null);

    m_shooterBottom.configure(
        m_shooterBottomConfig,
        ResetMode.kNoResetSafeParameters, null);
  }

  /**
   * Get top motor current
   */
  public double getTopMotorCurrent() {
    return m_shooterTop.getOutputCurrent();
  }

  /**
   * Stop the shooter
   */
  public void stopShooter() {
    m_shooterTop.set(0);
    m_shooterBottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Update the Elastic GUI Readouts for the shooter motors when in test mode //
    }
  }
}
