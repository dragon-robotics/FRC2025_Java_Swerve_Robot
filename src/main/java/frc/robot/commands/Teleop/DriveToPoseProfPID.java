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

  // Profiled-PID controllers for field-relative translate, strafe, and rotation
  // Tune these gains carefully!
  private final ProfiledPIDController m_translateController;
  private final ProfiledPIDController m_strafeController;
  private final ProfiledPIDController m_rotationController;

  // Linear and rotational Profiled-PID constraints //
  private final TrapezoidProfile.Constraints m_linearConstraints;
  private final TrapezoidProfile.Constraints m_rotationConstraints;

  // Tolerances - how close is close enough?
  private static final double m_positionTolerance = Units.inchesToMeters(1); // inches to meters
  private static final double m_angleTolerance = Math.toRadians(5.0); // degrees to radians

  /** Creates a new DriveToPoseProfPID. */
  public DriveToPoseProfPID(
      CommandSwerveDrivetrain swerve,
      ApplyRobotSpeeds robotSpeeds,
      Pose2d targetPose,
      TrapezoidProfile.Constraints linearConstraints,
      TrapezoidProfile.Constraints rotationConstraints) {
    m_swerve = swerve;
    m_robotSpeeds = robotSpeeds;
    m_targetPose = targetPose;

    m_linearConstraints = linearConstraints;
    m_rotationConstraints = rotationConstraints;

    // Initialize the PID controllers with constraints
    m_translateController = new ProfiledPIDController(
        4.0, 0, 0, // X controller with profiling
        m_linearConstraints);

    m_strafeController = new ProfiledPIDController(
        4.0, 0, 0, // Y controller with profiling
        m_linearConstraints);

    m_rotationController = new ProfiledPIDController(
        4.0, 0, 0, // Rotation controller
        m_rotationConstraints);

    // Set tolerances
    m_translateController.setTolerance(m_positionTolerance);
    m_strafeController.setTolerance(m_positionTolerance);
    m_rotationController.setTolerance(m_angleTolerance);

    // Allow rotation controller to wrap around (-pi to pi)
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = m_swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = m_swerve.getState().Speeds;
    
    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        currentSpeeds, currentPose.getRotation()
    );

    // Reset ProfiledPIDControllers with current position AND velocity
    m_translateController.reset(
        new TrapezoidProfile.State(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond)
    );
    m_strafeController.reset(
        new TrapezoidProfile.State(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond)
    );
    m_rotationController.reset(
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
    double xSpeed = m_translateController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = m_strafeController.calculate(currentPose.getY(), m_targetPose.getY());
    // Use current rotation for theta calculation's measurement
    double thetaSpeed = m_rotationController.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    // Debugging: Print the current pose and target pose
    // System.out.printf("Current Pose: %s, Target Pose: %s%n", currentPose, m_targetPose);
    // System.out.printf("X Speed: %.2f, Y Speed: %.2f, Theta Speed: %.2f%n", xSpeed, ySpeed, thetaSpeed);

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
    return m_translateController.atGoal() && m_strafeController.atGoal() && m_rotationController.atGoal();
  }
}
