// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveAlgaeManually extends Command {

  private AlgaeSubsystem m_algae;
  private DoubleSupplier m_armSpeed;
  private DoubleSupplier m_intakeSpeed;

  /** Creates a new MoveAlgaeArmManually. */
  public MoveAlgaeManually(
      AlgaeSubsystem algae,
      DoubleSupplier armSpeed,
      DoubleSupplier intakeSpeed) {
    m_algae = algae;
    m_armSpeed = armSpeed;
    m_intakeSpeed = intakeSpeed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armMotorSpeed = MathUtil.applyDeadband(m_armSpeed.getAsDouble(), 0.1);
    double intakeMotorSpeed = MathUtil.applyDeadband(m_intakeSpeed.getAsDouble(), 0.1);
    m_algae.setAlgaeArmMotorSpeed(armMotorSpeed);
    m_algae.setAlgaeIntakeMotorSpeed(intakeMotorSpeed);
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
