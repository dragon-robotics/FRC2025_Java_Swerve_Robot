// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldArmToPosition extends PIDCommand {
  /** Creates a new HoldArmToPosition. */

  public HoldArmToPosition(ArmSubsystem arm) {
    super(
        // The controller that the command will use
        new PIDController(2, 0, 0),
        // This should return the measurement
        () -> arm.getArmPosition(),
        // This should return the setpoint (can also be a constant)
        () -> arm.getLastSetpoint(),
        // This uses the output
        output -> {
          // Use the output here
          arm.setArmSpeed(output);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.01);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
