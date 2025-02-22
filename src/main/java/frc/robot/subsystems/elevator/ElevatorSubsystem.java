// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public boolean isAtElevatorState() {
    return switch(m_elevatorState){
      case L1 -> MathUtil.isNear(m_elevatorIO.getElevatorSetpoint(), L1, 0.01);
      case L2 -> MathUtil.isNear(m_elevatorIO.getElevatorSetpoint(), L2, 0.01);
      case L3 -> MathUtil.isNear(m_elevatorIO.getElevatorSetpoint(), L3, 0.01);
      case L4 -> MathUtil.isNear(m_elevatorIO.getElevatorSetpoint(), L4, 0.01);
      case HOME -> MathUtil.isNear(m_elevatorIO.getElevatorSetpoint(), HOME, 0.01);
      default -> false;
    };
  }

  /**
   * Set the height of the elevator
   * @param wantedElevatorState
   */
  public void setElevatorState(ElevatorState wantedElevatorState) {

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

  public void setElevatorMotorSpeed(double speed) {
    m_elevatorIO.setElevatorMotorPercentage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update inputs
    m_elevatorIO.updateInputs(m_elevatorIOInputs);
  }
}
