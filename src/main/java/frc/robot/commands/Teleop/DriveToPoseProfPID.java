// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPoseProfPID extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final ApplyRobotSpeeds m_robotSpeeds;
  private final Pose2d m_targetPose;

  private final Timer m_timer; // Used for ProfiledPIDController timing

  // PID controllers for field-relative X, Y, and Theta
  // Tune these gains carefully!
  // Profiled-PID controllers for X, Y, and rotation //
  private final ProfiledPIDController m_xController = new ProfiledPIDController(
      5.0, 0, 0, // X controller with profiling
      new TrapezoidProfile.Constraints(
          4.0, // Max velocity in X (m/s)
          8.0 // Max acceleration in X (m/s²)
      ));

  private final ProfiledPIDController m_yController = new ProfiledPIDController(
      5.0, 0, 0, // X controller with profiling
      new TrapezoidProfile.Constraints(
          4.0, // Max velocity in X (m/s)
          8.0 // Max acceleration in X (m/s²)
      ));

  private final ProfiledPIDController m_rotController = new ProfiledPIDController(
      5.0, 0, 0, // Rotation controller
      new TrapezoidProfile.Constraints(
          Units.degreesToRadians(540),
          Units.degreesToRadians(720)));

  // Tolerances - how close is close enough?
  private static final double m_positionTolerance = 0.05; // meters
  private static final double m_angleTolerance = Math.toRadians(2.0); // radians

  /** Creates a new DriveToPoseProfPID. */
  public DriveToPoseProfPID(CommandSwerveDrivetrain swerve, ApplyRobotSpeeds robotSpeeds, Pose2d targetPose) {
    m_swerve = swerve;
    m_robotSpeeds = robotSpeeds;
    m_targetPose = targetPose;

    // Allow rotation controller to wrap around (-pi to pi)
    m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances
    m_xController.setTolerance(m_positionTolerance);
    m_yController.setTolerance(m_positionTolerance);
    m_rotController.setTolerance(m_angleTolerance);

    // Initialize timer
    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Reset and start the timer
    m_timer.restart();

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = m_swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = m_swerve.getState().Speeds;
    
    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        currentSpeeds, currentPose.getRotation()
    );

    // Reset ProfiledPIDControllers with current position AND velocity
    m_xController.reset(
        new TrapezoidProfile.State(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond)
    );
    m_yController.reset(
        new TrapezoidProfile.State(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond)
    );
    m_rotController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), 
            fieldRelativeSpeeds.omegaRadiansPerSecond
        )
    );

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current pose from the estimator
    Pose2d currentPose = m_swerve.getState().Pose;

    // Calculate field-relative speeds using PID controllers
    double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
    // Use current rotation for theta calculation's measurement
    double thetaSpeed = m_rotController.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    // Convert field-relative speeds to robot-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, thetaSpeed, currentPose.getRotation()
    );

    // Command the swerve drive
    m_swerve.setControl(m_robotSpeeds.withSpeeds(targetSpeeds));    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    m_swerve.setControl(m_robotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0))); // Send zero speeds
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if all controllers are at their setpoints (within tolerance)
    return m_xController.atGoal() && m_yController.atGoal() && m_rotController.atGoal();
  }
}
