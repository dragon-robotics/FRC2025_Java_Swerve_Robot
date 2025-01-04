// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class TestArmSetpoints extends Command {
  private final ArmSubsystem m_arm;
  private final DoubleSupplier m_power;

  /** Creates a new TestArmSetpoints. */
  public TestArmSetpoints(ArmSubsystem arm, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    m_arm = arm;
    m_power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_power.getAsDouble();
    if (m_power.getAsDouble() < -0.2)
      speed = -0.2;
    else if (m_power.getAsDouble() > 0.2)
      speed = 0.2;
    m_arm.setArmSpeed(speed);
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
