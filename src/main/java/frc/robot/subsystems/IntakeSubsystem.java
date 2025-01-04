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
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Intake Subsystem //

  // Intake Motor Controller //
  private final SparkMax m_intake = new SparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

  // Intake Motor Controller Configurations //
  private final SparkBaseConfig m_intakeConfig = new SparkMaxConfig();

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {

    // Intake Motor Configuration //
    m_intakeConfig
        .voltageCompensation(IntakeConstants.NOMINAL_VOLTAGE)
        .smartCurrentLimit(IntakeConstants.STALL_CURRENT_LIMIT, IntakeConstants.FREE_CURRENT_LIMIT)
        .secondaryCurrentLimit(IntakeConstants.STALL_CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(IntakeConstants.RAMP_RATE_IN_SEC);

    // Create Shuffleboard entries for the IntakeSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Enable Elastic GUI Readouts for the intake motors in test mode //
    } else {
      m_intakeConfig.signals
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
    m_intake.configure(
        m_intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Set the intake to intake or outtake
   * @param intake true to intake, false to outtake
   */
  public void setIntake(boolean intake) {
    if (intake) {
      m_intake.set(0.3);
    } else {
      m_intake.set(-0.3);
    }
  } 

  /**
   * Set the intake to a specific speed
   * @param speed the speed to set the intake to
   */
  public void set(double speed) {
    m_intake.set(speed);
  }

  /**
   * Set the intake to a specific voltage
   * @param voltage the voltage to set the intake to
   */
  public void setVoltage(double voltage) {
    m_intake.setVoltage(voltage);
  }

  /**
   * Stop the intake
   */
  public void stopIntake() {
    m_intake.set(0.0);
  }

  /**
   * Get the current of the intake motor
   */
  public double getCurrent() {
    return m_intake.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      // @TODO: Update the Elastic GUI Readouts for the intake motors when in test mode //
    }
  }
}
