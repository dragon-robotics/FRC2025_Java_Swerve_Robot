package frc.robot.commands.Teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockHeading extends PIDCommand {
  public LockHeading(SwerveDriveSubsystem drive, double desiredAngle) {
    super(
      new PIDController(0.01, 0, 0),  // Replace with your PID gains
      drive::getYaw,  // Current heading
      desiredAngle,  // Desired heading
      correction -> {
        // Use the output here
        ChassisSpeeds desiredSpeeds
          = drive.swerve.swerveController.getTargetSpeeds(
              0,
              0,
              correction,
              drive.swerve.getYaw().getRadians(),
              drive.swerve.swerveController.config.maxAngularVelocity
          );

        drive.driveHeading(desiredSpeeds);
      },
      drive  // Required subsystem
    );

    getController().enableContinuousInput(-180, 180);  // Assumes angles are in the range -180 to 180

    getController().setTolerance(0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}