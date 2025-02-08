// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.RobotContainer;
import frc.robot.swerve_constant.TunerConstants;

public class Superstructure extends SubsystemBase {

  private AlgaeSubsystem m_algae;
  private CommandSwerveDrivetrain m_swerve;
  private CoralSubsystem m_coral;
  private ElevatorSubsystem m_elevator;
  private RobotContainer m_container;

  public enum WantedSuperState {
    STOPPED,  // Default state
    INTAKE_CORAL_LEFT,
    INTAKE_CORAL_RIGHT,
    INTAKE_ALGAE,
    ALGAE_STOWED,
    ALIGN_TO_REEF_LEFT,
    ALIGN_TO_REEF_RIGHT,
    ALIGN_TO_REEF_CENTER,
    HOME,
    L1,
    L2,
    L3,
    L4,
    SCORE_CORAL_TAG_6,
    SCORE_CORAL_TAG_7,
    SCORE_CORAL_TAG_8,
    SCORE_CORAL_TAG_9,
    SCORE_CORAL_TAG_10,
    SCORE_CORAL_TAG_11,
    SCORE_CORAL_TAG_17,
    SCORE_CORAL_TAG_18,
    SCORE_CORAL_TAG_19,
    SCORE_CORAL_TAG_20,
    SCORE_CORAL_TAG_21,
    SCORE_CORAL_TAG_22,
    SCORE_PROCESSOR
  }

  public enum CurrentSuperState {
    STOPPED,  // Default state
    INTAKE_CORAL_LEFT,
    INTAKE_CORAL_RIGHT,
    INTAKE_ALGAE,
    ALGAE_STOWED,
    ALIGN_TO_REEF_LEFT,
    ALIGN_TO_REEF_RIGHT,
    HOME,
    L1,
    L2,
    L3,
    L4,
    SCORE_CORAL_TAG_6,
    SCORE_CORAL_TAG_7,
    SCORE_CORAL_TAG_8,
    SCORE_CORAL_TAG_9,
    SCORE_CORAL_TAG_10,
    SCORE_CORAL_TAG_11,
    SCORE_CORAL_TAG_17,
    SCORE_CORAL_TAG_18,
    SCORE_CORAL_TAG_19,
    SCORE_CORAL_TAG_20,
    SCORE_CORAL_TAG_21,
    SCORE_CORAL_TAG_22,
    SCORE_PROCESSOR
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

    /** Creates a new Superstructure. */
    public Superstructure(
        CommandSwerveDrivetrain swerve,
        AlgaeSubsystem algae,
        CoralSubsystem coral,
        ElevatorSubsystem elevator,
        RobotContainer container) {
    m_swerve = swerve;
    m_algae = algae;
    m_coral = coral;
    m_swerve = swerve;
    m_elevator = elevator;
    m_container = container;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
