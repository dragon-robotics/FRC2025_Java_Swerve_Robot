// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class TestUptake extends Command {
  private final UptakeSubsystem m_uptake;
  private final DoubleSupplier m_speedSupplier;

  /** Creates a new TestUptake. */
  public TestUptake(UptakeSubsystem uptake, DoubleSupplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);

    m_uptake = uptake;
    m_speedSupplier = speedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_speedSupplier.getAsDouble();
    // if (speed < -0.9)
    //   speed = -0.9;
    // else if (speed > 0.9)
    //   speed = 0.9;
    m_uptake.set(speed);
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
