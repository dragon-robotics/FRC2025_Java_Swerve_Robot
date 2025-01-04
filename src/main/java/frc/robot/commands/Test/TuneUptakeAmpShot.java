// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class TuneUptakeAmpShot extends Command {

  private final UptakeSubsystem m_uptake;
  private final BooleanSupplier m_speedInc1Per;
  private final BooleanSupplier m_speedDec1per;
  private final BooleanSupplier m_shoot;
  private final BooleanSupplier m_reverse;

  private double m_speed;

  /** Creates a new TuneUptakeAmpShot. */
  public TuneUptakeAmpShot(
    UptakeSubsystem uptake, 
    BooleanSupplier speedInc1Per, 
    BooleanSupplier speedDec1per, 
    BooleanSupplier shoot, 
    BooleanSupplier reverse) {
    m_uptake = uptake;
    m_speedInc1Per = speedInc1Per;
    m_speedDec1per = speedDec1per;
    m_shoot = shoot;
    m_reverse = reverse;

    m_speed = -0.3;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_speedInc1Per.getAsBoolean()) {
      m_speed -= 0.01;
      System.out.println("Speed: " + m_speed);
    } else if(m_speedDec1per.getAsBoolean()) {
      m_speed += 0.01;
      System.out.println("Speed: " + m_speed);
    }

    if (m_speed < -0.6) {
      m_speed = -0.6;
    } else if (m_speed > -0.3) {
      m_speed = -0.3;
      
    }

    if(m_shoot.getAsBoolean()) {
      m_uptake.set(m_speed);
    } else if(m_reverse.getAsBoolean()) {
      m_uptake.set(0.15);
    } else {
      m_uptake.set(0);
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
