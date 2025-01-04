// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntakeUntilNoteDetected extends Command {
  
  private final IntakeSubsystem m_intake;
  private final DoubleSupplier m_speed;

  // Used to keep track of the time the command started //
  private long m_startTime;


  /** Creates a new MoveIntakeUntilNoteDetected. */
  public MoveIntakeUntilNoteDetected(
    IntakeSubsystem intake,
    DoubleSupplier speed
  ) {
    m_intake = intake;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.set(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the current is greater than a certain amount of amps
   */
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() - m_startTime < 200)
      return false;
    else
      // System.out.println("Current: " + m_intake.getCurrent());
      return m_intake.getCurrent() > IntakeConstants.NOTE_DETECT_CURRENT_THRESHOLD;
  }
}
