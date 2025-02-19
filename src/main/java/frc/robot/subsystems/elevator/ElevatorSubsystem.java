// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxSim;
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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import static frc.robot.Constants.ElevatorSubsystemConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

  // @TODO: Create Elastic Tabs for the Elevator Subsystem //
  
  // Algae Subsystem States //
  public enum WantedState {
    IDLE,
    HOME,
    L1,
    L2,
    L3,
    L4,
    OFF
  }

  public enum SystemState {
    IDLING,
    AT_HOME,
    AT_L1,
    AT_L2,
    AT_L3,
    AT_L4,
    OFF
  }

  private ElevatorIO m_elevatorIO;
  private ElevatorIOInputs m_elevatorIOInputs = new ElevatorIOInputs();

  private WantedState m_wantedState = WantedState.HOME;
  private SystemState m_systemState = SystemState.AT_HOME;

  // // Elevator Simulation //
  // private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
  // private final SparkMaxSim m_elevatorLeftSim = new SparkMaxSim(m_elevatorLeft, m_elevatorGearbox);
  // private final SparkMaxSim m_elevatorRightSim = new SparkMaxSim(m_elevatorRight, m_elevatorGearbox);

  // // Simulation classes help us simulate what's going on, including gravity.
  // private final ElevatorSim m_elevatorSim =
  //     new ElevatorSim(
  //         m_elevatorGearbox,
  //         ElevatorConstants.kElevatorGearing,
  //         ElevatorConstants.kCarriageMass,
  //         ElevatorConstants.kElevatorDrumRadius,
  //         ElevatorConstants.kMinElevatorHeightMeters,
  //         ElevatorConstants.kMaxElevatorHeightMeters,
  //         true,
  //         0,
  //         0.01,
  //         0.0);

  // // Create a Mechanism2d visualization of the elevator
  // private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 12);
  // private final MechanismRoot2d     m_mech2dRoot     = m_mech2d.getRoot("Elevator Root", 10, 0);
  // private final MechanismLigament2d m_elevatorMech2d =
  //     m_mech2dRoot.append(
  //         new MechanismLigament2d("Elevator", m_elevatorSim.getPositionMeters(), 90));

  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    m_elevatorIO = elevatorIO;
  }

  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return m_elevatorIOInputs.elevatorCurrentLimitTripped;
  }

  /**
   * Set the wanted state of the elevator
   * @param wantedState
   */
  public void setWantedState(WantedState wantedState) {
    m_wantedState = wantedState;
  }

  /**
   * Handle the state transition
   * @return the new state
   */
  private SystemState handleStateTransition() {
    return switch (m_wantedState) {
      case OFF -> SystemState.OFF;
      case L1 -> SystemState.AT_L1;
      case L2 -> SystemState.AT_L2;
      case L3 -> SystemState.AT_L3;
      case L4 -> SystemState.AT_L4;
      case HOME -> SystemState.AT_HOME;
      default -> SystemState.AT_HOME;
    };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_elevatorIO.updateInputs(m_elevatorIOInputs);

    // Get the new state //
    SystemState newState = handleStateTransition();
    if (newState != m_systemState) {
      m_systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      m_systemState = SystemState.AT_HOME;
    }

    // If the current limit is tripped (e.g. the elevator is at the top or bottom), stop the motor
    if (isCurrentLimitTripped()) {
      // If the elevator is at the bottom, set the encoder to 0 //
      if (m_elevatorIOInputs.elevatorLeadMotorPosition > HOME_GOAL) {
        m_elevatorIO.seedElevatorMotorEncoderPosition(0);
      }
      // Stop the motor on the elevator //
      m_systemState = SystemState.IDLING;
    }

    // Handle the state transition //
    switch (m_systemState) {
      case IDLING:
        m_elevatorIO.setElevatorMotorPercentage(0);
        break;
      case AT_HOME:
        m_elevatorIO.setElevatorMotorSetpoint(HOME_GOAL);
        break;
      case AT_L1:
        m_elevatorIO.setElevatorMotorSetpoint(L1);
        break;
      case AT_L2:
        m_elevatorIO.setElevatorMotorSetpoint(L2);
        break;
      case AT_L3:
        m_elevatorIO.setElevatorMotorSetpoint(L3);
        break;
      case AT_L4:
        m_elevatorIO.setElevatorMotorSetpoint(L4);
        break;
      case OFF:
        break;
      default:
        break;
    }
  }
}
