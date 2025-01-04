// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooter extends Command {
  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_forwardSpeedSupplier;
  private final DoubleSupplier m_reverseSpeedSupplier;

  /** Creates a new TestShooter. */
  public TestShooter(
      ShooterSubsystem shooter,
      DoubleSupplier forwardSpeedSupplier,
      DoubleSupplier reverseSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
    m_reverseSpeedSupplier = reverseSpeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = m_forwardSpeedSupplier.getAsDouble();
    double reverseSpeed = -m_reverseSpeedSupplier.getAsDouble();
    double speed = forwardSpeed + reverseSpeed;
    if (speed < -0.7)
      speed = -0.7;
    else if (speed > 0.7)
      speed = 0.7; 
    m_shooter.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
