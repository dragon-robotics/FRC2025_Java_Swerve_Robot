// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.AlgaeSubsystemConstants.*;

import java.util.function.BooleanSupplier;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // @TODO: Create Elastic Tabs for the Arm Subsystem //

  // Algae Subsystem States //
  public enum WantedState {
    IDLE,
    HOME,
    INTAKE,
    DEALGAE,
    HOLD,
    EJECT,
    OFF
  }

  public enum SystemState {
    IDLING,
    AT_HOME,
    INTAKING,
    DEALGAEING,
    HOLDING,
    EJECTING,
    OFF
  }

  private AlgaeIO m_algaeIO;
  private AlgaeIOInputs m_algaeIOInputs = new AlgaeIOInputs();

  private WantedState m_wantedState = WantedState.IDLE;
  private SystemState m_systemState = SystemState.IDLING;

  private boolean m_hasAlgae = false;


//   // SysID Routine for Arm Tuning //
//   // SysId Routine and seutp
//   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
//   private final MutVoltage         m_appliedVoltage = Volts.mutable(0);
//   // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
//   private final MutAngle           m_angle          = Rotations.mutable(0);
//   // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
//   private final MutAngularVelocity m_velocity       = RPM.mutable(0);
//   // SysID Routine
//   private final SysIdRoutine       m_sysIdRoutine   =
//       new SysIdRoutine(
//           // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
//           new SysIdRoutine.Config(Volts.per(Second).of(ARM_RAMP_RATE_IN_SEC), Volts.of(6), Seconds.of(30)),
//           new SysIdRoutine.Mechanism(
//               // Tell SysId how to plumb the driving voltage to the motor(s).
//               m_algaeIO::setArmMotorVoltage,
//               // Tell SysId how to record a frame of data for each motor on the mechanism being
//               // characterized.
//               log -> {
//                 // Record a frame for the arm motor.
//                 log.motor("arm")
//                    .voltage(
//                        m_appliedVoltage.mut_replace(m_algaeIOInputs.armLeftMotorCurrent *
//                                                     RobotController.getBatteryVoltage(), Volts))
//                    .angularPosition(m_angle.mut_replace(m_encoder.getPosition(), Rotations))
//                    .angularVelocity(m_velocity.mut_replace(m_encoder.getVelocity(), RPM));
// //                .angularPosition(m_angle.mut_replace(getAngle()))
// //                .angularVelocity(m_velocity.mut_replace(getVelocity()));
//               },
//               this));
  
  public AlgaeSubsystem(AlgaeIO algaeIO) {
    m_algaeIO = algaeIO;
  }
  
  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return m_algaeIOInputs.intakeCurrentLimitTripped;
  }

  /**
   * Get whether the mechanism holds an algae
   */
  public boolean hasAlgae() {
    return m_hasAlgae;
  }

  /**
   * Set the wanted state of the arm
   * @param wantedState
   */
  public void setWantedState(WantedState wantedState) {
    m_wantedState = wantedState;
  }

  /**
   * Get the current state of the coral intake
   * @return m_systemState
   */
  public SystemState getSystemState() {
    return m_systemState;
  }

  /**
   * Handle the state transition
   * @return the new state
   */
  private SystemState handleStateTransition() {
    return switch (m_wantedState) {
      case OFF -> SystemState.OFF;
      case EJECT -> SystemState.EJECTING;
      case DEALGAE -> SystemState.DEALGAEING;
      case HOLD -> SystemState.HOLDING;
      case INTAKE -> {
        if (isCurrentLimitTripped()) {
          yield SystemState.HOLDING;
        }
        yield SystemState.INTAKING;
      }
      default -> SystemState.AT_HOME;
    };
  }

  /**
   * Handles the action of the intake and the arm motors for each state
   */
  public void handleMotors(double armSetpoint, double intakeSpeed) {
    m_algaeIO.setArmSetpoint(armSetpoint);
    m_algaeIO.setIntakeMotorPercentage(intakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_algaeIO.updateInputs(m_algaeIOInputs);
    
    // Get the new state //
    SystemState newState = handleStateTransition();
    if(newState != m_systemState) {
      m_systemState = newState;
    }

    // Stop moving when the robot is disabled //
    if (DriverStation.isDisabled()) {
      m_systemState = SystemState.IDLING;
    }

    // Handle the state transition //
    // set voltages based on state
    switch (m_systemState) {
      case EJECTING:
        // Hold the arm at the ejecting position and spin the intake to eject the algae //
        handleMotors(ARM_PROCESSOR_OUTTAKE_GOAL, OUTTAKE_SPEED);    
        m_hasAlgae = false;
        
        break;
      case INTAKING:
        // Hold the arm at the INTAKing position and spin the intake to INTAKE the algae //  
        handleMotors(ARM_INTAKE_GOAL, INTAKE_SPEED);
        break;
      case DEALGAEING:
        // Hold the arm at the dealgaeing position and spin the intake to dealgae the algae //
        handleMotors(ARM_DEALGAE_GOAL, DEALGAE_SPEED);
        break;
      case HOLDING:
        // Hold the arm at the holding position and stop the intake from spinning //
        handleMotors(ARM_HOLD_GOAL, 0);
        m_hasAlgae = true;
        break;
      case AT_HOME:
        // Hold the arm at the home position and stop the intake from spinning //
        handleMotors(ARM_HOME_GOAL, 0);
        break;
      case IDLING:
        // Hold the arm at the home position //
        m_algaeIO.setArmMotorVoltage(0);

        // Stop the intake motor from spinning //
        m_algaeIO.setIntakeMotorVoltage(0);
        break;
      case OFF:
        break;
      default:
        // Hold the arm at the home position and stop the intake from spinning //
        handleMotors(ARM_HOME_GOAL, 0);
        break;
      }
  }
}
