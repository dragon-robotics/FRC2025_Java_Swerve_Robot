// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtSpeaker extends ProfiledPIDCommand {

  /** Creates a new AimAtTarget. */
  public AimAtSpeaker(SwerveDriveSubsystem swerve) {

    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.1, 0.1)),
        // This should return the measurement
        () -> LimelightHelpers.getTX("Limelight 1"),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(0, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          swerve.drive(new Translation2d(0,0).times(SwerveConstants.MAX_SPEED_METERS_PER_SECOND), output, true, false);
        });

        // Something to figure out whether we're red or blue to determine if we're using #4 or #7 apriltags
    Optional<Alliance> alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  
    int targetToUse = isRed ? 4 : 7;
  
    // Change the pipeline to the target we want to use //
    LimelightHelpers.setPipelineIndex("Limelight 1", targetToUse);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.05);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
