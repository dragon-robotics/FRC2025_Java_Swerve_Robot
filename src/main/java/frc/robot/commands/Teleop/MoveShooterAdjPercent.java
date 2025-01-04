// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterAdjPercent extends Command {

  private final ShooterSubsystem m_shooter;
  private final BooleanSupplier m_activate;
  private final BooleanSupplier m_increaseByOnePercent;
  private final BooleanSupplier m_decreaseByOnePercent;

  // Adjustable speed for the shooter - starts at 0% //
  private double m_speed;

  /** Creates a new MoveShooterAdjPercent. */
  public MoveShooterAdjPercent(
    ShooterSubsystem shooter,
    BooleanSupplier activate,
    BooleanSupplier increaseByOnePercent,
    BooleanSupplier decreaseByOnePercent) {

    // Speed starts at 0% //
    m_speed = 0;

    m_shooter = shooter;
    m_activate = activate;
    m_increaseByOnePercent = increaseByOnePercent;
    m_decreaseByOnePercent = decreaseByOnePercent;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_activate.getAsBoolean()) {
      if (m_increaseByOnePercent.getAsBoolean()) {
        m_speed += 0.01;
        if (m_speed > 1) m_speed = 1;
        else if (m_speed < -1) m_speed = -1;
        else if (m_speed < 0.25) m_speed = -0.25;
        else if (m_speed > -0.25) m_speed = 0.25;  
        System.out.println("m_speed: " + m_speed);
      } else if (m_decreaseByOnePercent.getAsBoolean()) {
        m_speed -= 0.01;
        if (m_speed > 1) m_speed = 1;
        else if (m_speed < -1) m_speed = -1;
        else if (m_speed < 0.25) m_speed = -0.25;
        else if (m_speed > -0.25) m_speed = 0.25;  
        System.out.println("m_speed: " + m_speed);
      }
      m_shooter.set(m_speed);
    }
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
