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
  public enum ElevatorState {
    IDLE,
    HOME,
    L1,
    L2,
    L3,
    L4,
  }

  private ElevatorIO m_elevatorIO;
  private ElevatorState m_elevatorState;
  private ElevatorIOInputs m_elevatorIOInputs;

  /** Creates a new ClimberSubsystem. */
  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    m_elevatorIO = elevatorIO;
    m_elevatorState = ElevatorState.IDLE;
    m_elevatorIOInputs = new ElevatorIOInputs();
  }

  /*
   * Check if current limit is tripped
   * @return true if current limit is tripped
   */
  public boolean isCurrentLimitTripped() {
    return m_elevatorIOInputs.elevatorCurrentLimitTripped;
  }

  /**
   * Set the height of the elevator
   * @param wantedElevatorState
   */
  public void setHeight(ElevatorState wantedElevatorState) {

    m_elevatorState = wantedElevatorState;

    switch(m_elevatorState){
      case L1:
        m_elevatorIO.setElevatorMotorSetpoint(L1);
        break;
      case L2:
        m_elevatorIO.setElevatorMotorSetpoint(L2);
        break;
      case L3:
        m_elevatorIO.setElevatorMotorSetpoint(L3);
        break;
      case L4:
        m_elevatorIO.setElevatorMotorSetpoint(L4);
        break;
      case HOME:
        m_elevatorIO.setElevatorMotorSetpoint(HOME);
        break;
      case IDLE:
        // Stop the motors if the wanted state is IDLE //
        m_elevatorIO.setElevatorMotorPercentage(0);
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_elevatorIO.updateInputs(m_elevatorIOInputs);
  }
}
