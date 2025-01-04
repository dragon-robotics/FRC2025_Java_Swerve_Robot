// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterUntilNoteDetected extends Command {

  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_speed;

  // Used to keep track of the time the command started //
  private long m_startTime;

  /** Creates a new MoveShooterUntilNoteDetected. */
  public MoveShooterUntilNoteDetected(
    ShooterSubsystem shooter,
    DoubleSupplier speed
  ) {
    m_shooter = shooter;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.set(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(System.currentTimeMillis() - m_startTime < 200)
      return false;
    else
      return m_shooter.getTopMotorCurrent() > ShooterConstants.NOTE_DETECT_CURRENT_THRESHOLD;
  }
}
