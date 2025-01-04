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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class UptakeSubsystem extends SubsystemBase {
  /** Creates a new UptakeSubsystem. */
  
  // @TODO: Create Elastic Tabs for the Uptake Subsystem //

  // Uptake Motor Controllers //
  private final SparkMax m_uptakeLeft = new SparkMax(UptakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax m_uptakeRight = new SparkMax(UptakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  // Uptake Motor Configurations //
  private final SparkBaseConfig m_uptakeLeftConfig = new SparkMaxConfig();
  private final SparkBaseConfig m_uptakeRightConfig = new SparkMaxConfig();

  private final DigitalInput m_noteBeamBreak = new DigitalInput(UptakeConstants.NOTE_BEAM_BREAK_DIGITAL_CHANNEL);

  /**
   * Creates a new UptakeSubsystem.
   */
  public UptakeSubsystem() {

    // Left Motor Configuration //
    m_uptakeLeftConfig
        .voltageCompensation(UptakeConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(UptakeConstants.STALL_CURRENT_LIMIT, UptakeConstants.FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(UptakeConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(UptakeConstants.RAMP_RATE_IN_SEC);

    // Right Motor Configuration //
    m_uptakeRightConfig
        .voltageCompensation(UptakeConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(UptakeConstants.STALL_CURRENT_LIMIT, UptakeConstants.FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(UptakeConstants.SECONDARY_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(UptakeConstants.RAMP_RATE_IN_SEC)
        .follow(UptakeConstants.LEFT_MOTOR_ID);

    // Create Shuffleboard entries for the UptakeSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the uptake motors in test mode //
    } else {
      m_uptakeLeftConfig.signals
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

      m_uptakeRightConfig.signals
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

    // Finalize Configuration //
    m_uptakeLeft.configure(
      m_uptakeLeftConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_uptakeRight.configure(
      m_uptakeRightConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Set the motors initially to 0 speed //
    m_uptakeLeft.set(0.0);
    m_uptakeRight.set(0.0);
    
  }

  public void setSpeedForward100() {
    m_uptakeLeft.set(1);
  }

  public void setSpeedReverse100() {
    m_uptakeLeft.set(-1);
  }

  public void setSpeed0() {
    m_uptakeLeft.set(0);
  }

  /**
   * Determine if a note is detected by the uptake note beam break sensor
   * @return true if a note is detected, false if not
   */
  public boolean isNoteDetected() {
    return m_noteBeamBreak.get();
  }

  /**
   * Set the intake to uptake or downtake
   * @param uptake true to uptake, false to downtake
   */
  public void setUptake(boolean uptake) {
    if (uptake) {
      m_uptakeLeft.set(1.0);
    } else {
      m_uptakeLeft.set(-1.0);
    }
  } 

  /**
   * Set the uptake to a specific speed
   * @param speed the speed to set the uptake to
   */
  public void set(double speed) {
    m_uptakeLeft.set(speed);
  }

  /**
   * Set the uptake to a specific voltage
   * @param voltage the voltage to set the uptake to
   */
  public void setVoltage(double voltage) {
    m_uptakeLeft.setVoltage(voltage);
  }

  /**
   * Set the uptake to uptake at 100% speed
   */
  public void uptake100() {
    m_uptakeLeft.set(1);
  }

  /**
   * Set the uptake to uptake at 40% speed
   */
  public void uptake40() {
    m_uptakeLeft.set(0.4);
  }

  /**
   * Set the uptake to downtake at 100% speed
   */
  public void downtake100() {
    m_uptakeLeft.set(-1);
  }

  /**
   * Set the uptake to downtake at 40% speed
   */
  public void downtake40() {
    m_uptakeLeft.set(-0.4);
  }

  /**
   * Stop the uptake
   */
  public void stopUptake() {
    m_uptakeLeft.set(0.0);
  }

  @Override
  public void periodic() {
    // System.out.println(isNoteDetected());

    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Update the Elastic GUI Readouts for the uptake motors when in test mode //
    }
  }
}
