// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseTrajPID extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final ApplyRobotSpeeds robotSpeeds;

  private Timer timer;
  private Trajectory trajectory;

  // Profiled-PID controllers for X, Y, and rotation //
  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          4.0,
          0,
          0, // X controller with profiling
          new TrapezoidProfile.Constraints(
              4.0, // Max velocity in X (m/s)
              8.0 // Max acceleration in X (m/s²)
              ));

  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          4.0,
          0,
          0, // X controller with profiling
          new TrapezoidProfile.Constraints(
              4.0, // Max velocity in X (m/s)
              8.0 // Max acceleration in X (m/s²)
              ));

  private final ProfiledPIDController rotController =
      new ProfiledPIDController(
          5.0,
          0,
          0, // Rotation controller
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(540), Units.degreesToRadians(720)));

  /** Creates a new DriveToPoseTrajPID. */
  public DriveToPoseTrajPID(
      CommandSwerveDrivetrain swerve,
      ApplyRobotSpeeds robotSpeeds,
      List<Pose2d> waypoints,
      boolean reverse) {
    this.swerve = swerve;
    this.robotSpeeds = robotSpeeds;

    // Allow rotation controller to wrap around (-pi to pi)
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the tolerance of the controller //
    xController.setTolerance(0.01); // 10 cm tolerance for X
    yController.setTolerance(0.01); // 10 cm tolerance for Y
    rotController.setTolerance(Units.degreesToRadians(2)); // 5 degrees tolerance for rotation

    // Initialize timer
    timer = new Timer();

    // Generate trajectory (same as above)
    TrajectoryConfig config = new TrajectoryConfig(1.5, 3.0);
    config.setReversed(reverse); // Set to true if you want to reverse the trajectory
    trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Reset and start the timer
    timer.restart();

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = swerve.getState().Speeds;

    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

    // Reset ProfiledPIDControllers with current position AND velocity
    xController.reset(
        new TrapezoidProfile.State(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond));
    yController.reset(
        new TrapezoidProfile.State(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond));
    rotController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Execute - follow trajectory with PID
    double currentTime = timer.get();
    Trajectory.State desiredState = trajectory.sample(currentTime);
    Pose2d currentPose = swerve.getState().Pose;

    // Calculate desired chassis speeds using PID
    double xSpeed = xController.calculate(currentPose.getX(), desiredState.poseMeters.getX());
    double ySpeed = yController.calculate(currentPose.getY(), desiredState.poseMeters.getY());
    double rotSpeed =
        rotController.calculate(
            currentPose.getRotation().getRadians(),
            desiredState.poseMeters.getRotation().getRadians());

    // Convert field-relative speeds to robot-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, currentPose.getRotation());

    // Command the swerve drive
    swerve.setControl(robotSpeeds.withSpeeds(targetSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds())); // Send zero speeds
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer.get() >= trajectory.getTotalTimeSeconds(); // Finished when trajectory
    // complete
    return xController.atGoal() && yController.atGoal() && rotController.atGoal();
  }
}
