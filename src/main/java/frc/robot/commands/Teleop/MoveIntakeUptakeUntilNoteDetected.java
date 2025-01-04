// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

public class MoveIntakeUptakeUntilNoteDetected extends Command {

  // @TODO: x 1. Make sure to clean the sensors
  // @TODO:   2. Make sure to position the sensors differently
  // @TODO:   3. Use the 5mm sensor instead - Change the sensor
  // @TODO: x 4. Use threads to speed up update rate
  // @TODO: x 5. Make the shooter detect a current spike
  // @TODO:   6. Use Voltage to set the motor speeds instead of using percentages
  // @TODO:   7. If the timing is consistent, extend the sequence by a little bit

  private IntakeSubsystem m_intake;
  private UptakeSubsystem m_uptake;
  private DoubleSupplier m_intakeVoltage;
  private DoubleSupplier m_uptakeVoltage;

  /** Creates a new MoveUptakeUntilNoteDetected. */
  public MoveIntakeUptakeUntilNoteDetected(
    IntakeSubsystem intake,
    UptakeSubsystem uptake,
    DoubleSupplier intakeVoltage,
    DoubleSupplier uptakeVoltage) {

    m_intake = intake;
    m_uptake = uptake;
    m_uptakeVoltage = uptakeVoltage;
    m_intakeVoltage = intakeVoltage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setVoltage(m_intakeVoltage.getAsDouble());
    m_uptake.setVoltage(m_uptakeVoltage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltage(0);
    m_uptake.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_uptake.isNoteDetected();
  }
}
