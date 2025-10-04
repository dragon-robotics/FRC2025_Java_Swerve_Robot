// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseProfPID extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final ApplyRobotSpeeds robotSpeeds;
  private final Pose2d targetPose;

  // Profiled-PID controllers for field-relative translate, strafe, and rotation
  // Tune these gains carefully!
  private final ProfiledPIDController translateController;
  private final ProfiledPIDController strafeController;
  private final ProfiledPIDController rotationController;

  // Linear and rotational Profiled-PID constraints //
  private final TrapezoidProfile.Constraints linearConstraints;
  private final TrapezoidProfile.Constraints rotationConstraints;

  // Tolerances - how close is close enough?
  private static final double POSITION_TOLERANCE = Units.inchesToMeters(2); // inches to meters
  private static final double ANGLE_TOLERANCE = Math.toRadians(2.0); // degrees to radians

  /** Creates a new DriveToPoseProfPID. */
  public DriveToPoseProfPID(
      CommandSwerveDrivetrain swerve,
      ApplyRobotSpeeds robotSpeeds,
      Pose2d targetPose,
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints rotationConstraints) {
    this.swerve = swerve;
    this.robotSpeeds = robotSpeeds;
    this.targetPose = targetPose;

    this.linearConstraints = linearConstraints;
    this.rotationConstraints = rotationConstraints;

    // Initialize the PID controllers with constraints
    translateController =
        new ProfiledPIDController(
            4.0,
            0,
            0, // X controller with profiling
            this.linearConstraints);

    strafeController =
        new ProfiledPIDController(
            4.0,
            0,
            0, // Y controller with profiling
            this.linearConstraints);

    rotationController =
        new ProfiledPIDController(
            4.0,
            0,
            0, // Rotation controller
            this.rotationConstraints);

    // Set tolerances
    translateController.setTolerance(POSITION_TOLERANCE);
    strafeController.setTolerance(POSITION_TOLERANCE);
    rotationController.setTolerance(ANGLE_TOLERANCE);

    // Allow rotation controller to wrap around (-pi to pi)
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = swerve.getState().Speeds;

    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

    // Reset ProfiledPIDControllers with current position AND velocity
    translateController.reset(
        new TrapezoidProfile.State(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond));
    strafeController.reset(
        new TrapezoidProfile.State(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond));
    rotationController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current pose from the estimator
    Pose2d currentPose = swerve.getState().Pose;

    // Calculate field-relative speeds using PID controllers
    double xSpeed = translateController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = strafeController.calculate(currentPose.getY(), targetPose.getY());
    // Use current rotation for theta calculation's measurement
    double thetaSpeed =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Convert field-relative speeds to robot-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, currentPose.getRotation());

    // Command the swerve drive
    swerve.setControl(robotSpeeds.withSpeeds(targetSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0))); // Send zero speeds
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if all controllers are at their setpoints (within tolerance)
    return translateController.atGoal()
        && strafeController.atGoal()
        && rotationController.atGoal();
  }
}
