package frc.robot.commands.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Your swerve subsystem

/** Drives the robot to a specified pose on the field using PID controllers. */
public class DriveToPosePID extends Command {
  private final CommandSwerveDrivetrain m_swerve;
  private final ApplyRobotSpeeds m_robotSpeeds;
  private final Pose2d m_targetPose;

  // PID controllers for field-relative X, Y, and Theta
  // Tune these gains carefully!
  private final PIDController m_xController = new PIDController(4, 0, 0); // P_x
  private final PIDController m_yController = new PIDController(4, 0, 0); // P_y
  private final ProfiledPIDController m_rotController =
      new ProfiledPIDController(
          5.0,
          0,
          0, // Rotation controller
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(540), Units.degreesToRadians(720)));

  // Tolerances - how close is close enough?
  private static final double m_positionTolerance = 0.05; // meters
  private static final double m_angleTolerance = Math.toRadians(2.0); // radians

  public DriveToPosePID(
      CommandSwerveDrivetrain swerve, ApplyRobotSpeeds robotSpeeds, Pose2d targetPose) {
    m_swerve = swerve;
    m_robotSpeeds = robotSpeeds;
    m_targetPose = targetPose;

    // Allow rotation controller to wrap around (-pi to pi)
    m_rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances
    m_xController.setTolerance(m_positionTolerance);
    m_yController.setTolerance(m_positionTolerance);
    m_rotController.setTolerance(m_angleTolerance);

    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = m_swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = m_swerve.getState().Speeds;

    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

    // Reset controllers if needed (though calculate usually handles this)
    m_xController.reset();
    m_yController.reset();
    m_rotController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond));
  }

  @Override
  public void execute() {
    // Get current pose from the estimator
    Pose2d currentPose = m_swerve.getState().Pose;

    // Calculate field-relative speeds using PID controllers
    double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
    // Use current rotation for theta calculation's measurement
    double thetaSpeed =
        m_rotController.calculate(
            currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    // Convert field-relative speeds to robot-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, currentPose.getRotation());

    // Command the swerve drive
    m_swerve.setControl(m_robotSpeeds.withSpeeds(targetSpeeds));
  }

  @Override
  public boolean isFinished() {
    // Check if all controllers are at their setpoints (within tolerance)
    return m_xController.atSetpoint() && m_yController.atSetpoint() && m_rotController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    m_swerve.setControl(m_robotSpeeds.withSpeeds(new ChassisSpeeds())); // Send zero speeds
  }
}
