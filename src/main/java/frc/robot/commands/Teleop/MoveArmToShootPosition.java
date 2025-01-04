// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToShootPosition extends ProfiledPIDCommand {
  /** Creates a new MoveArmToShootPositionPID. */

  private ArmSubsystem m_arm;

  public MoveArmToShootPosition(ArmSubsystem arm) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            7,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(10, 20)
        ),
        // This should return the measurement
        () -> arm.getArmPosition(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(ArmConstants.SHOOTER_GOAL, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          System.err.println("Output: " + output + " Setpoint: " + setpoint.position);
          arm.setArmSpeed(output);
        },
        arm
    );
    
    m_arm = arm;
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.005);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setLastSetpoint(getController().getGoal().position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("At setpoint?: " + getController().atGoal());
    // System.out.println("The sepoint is: " + getController().getSetpoint().position);
    
    return getController().atGoal();
  }
}
