package frc.robot.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Your swerve subsystem
import frc.robot.swerve_constant.TunerConstants;

/** Drives the robot to a specified pose on the field using PID controllers. */
public class DriveToPosePID extends Command {
  private final CommandSwerveDrivetrain swerve;
  private final ApplyRobotSpeeds robotSpeeds;
  private final Pose2d targetPose;

  // PID controllers for field-relative X, Y, and Theta
  // Tune these gains carefully!
  private final PIDController xController = new PIDController(7, 0, 0); // P_x
  private final PIDController yController = new PIDController(7, 0, 0); // P_y
  private final ProfiledPIDController rotController =
      new ProfiledPIDController(
          5,
          0,
          0, // Rotation controller
          new TrapezoidProfile.Constraints(
              Units.degreesToRadians(540), Units.degreesToRadians(720)));

  // Tolerances - how close is close enough?
  private static final double POSITION_TOLERANCE = Units.inchesToMeters(1); // meters
  private static final double ANGLE_TOLERANCE = Math.toRadians(2); // radians

  public DriveToPosePID(
      CommandSwerveDrivetrain swerve, ApplyRobotSpeeds robotSpeeds, Pose2d targetPose) {
    this.swerve = swerve;
    this.robotSpeeds = robotSpeeds;
    this.targetPose = targetPose;

    // Allow rotation controller to wrap around (-pi to pi)
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    rotController.setTolerance(ANGLE_TOLERANCE);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {

    // Get the current pose and velocity of the swerve drive
    Pose2d currentPose = swerve.getState().Pose;
    ChassisSpeeds currentSpeeds = swerve.getState().Speeds;

    // Convert robot-relative speeds to field-relative
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

    // Reset controllers if needed (though calculate usually handles this)
    xController.reset();
    yController.reset();
    rotController.reset(
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond));
  }

  @Override
  public void execute() {
    // Get current pose from the estimator
    Pose2d currentPose = swerve.getState().Pose;

    // Calculate field-relative speeds using PID controllers
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    // Use current rotation for theta calculation's measurement
    double thetaSpeed =
        rotController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Clamp linear speeds to 75% of max speed for safety
    double maxLinearSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    xSpeed = MathUtil.clamp(xSpeed, -0.5 * maxLinearSpeed, 0.5 * maxLinearSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -0.5 * maxLinearSpeed, 0.5 * maxLinearSpeed);

    // Convert field-relative speeds to robot-relative ChassisSpeeds
    ChassisSpeeds targetSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, thetaSpeed, currentPose.getRotation());

    // Command the swerve drive
    swerve.setControl(robotSpeeds.withSpeeds(targetSpeeds));
  }

  @Override
  public boolean isFinished() {
    // Check if all controllers are at their setpoints (within tolerance)
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command finishes or is interrupted
    swerve.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds())); // Send zero speeds
  }
}
