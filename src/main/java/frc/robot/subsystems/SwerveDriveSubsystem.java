// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final SwerveDrive swerve;

  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
    /* 
     * Initialize the swerve drive based on the
     * configuration files deployed to the RoboRIO
     */
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

    try {
      swerve = new SwerveParser(
        new File(Filesystem.getDeployDirectory(), "swerve"))
        .createSwerveDrive(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

      // Configure the AutoBuilder //
      setupPathPlanner();
        
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerve.setOdometryPeriod(0.01);
    swerve.setHeadingCorrection(true);
    swerve.setCosineCompensator(true);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // @TODO: Figure out why the intake sequence doesn't work initially sometimes //
    // @TODO: Add the uptake shot sequence to improve our uptake shot consistency (BTN 11) //
    // @TODO: Slow down the uptake shot to have the shot be lobbed in //
    // @TODO: Have the return sequences on the 4-note auto go a bit further into the subwoofer to align our shots better //
    // @TODO: Sweeping motion on 4-note auto to clear out missed notes //

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Drive Command
   *
   * @return the drive command
   */
  public Command drive(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier halfSpeedSup
  ) {
    return run(() -> {
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();
        double rotation = rotationSup.getAsDouble();
        boolean halfSpeed = halfSpeedSup.getAsBoolean();

        // Apply half speed if the half speed button is pressed
        if(halfSpeed)
        {

          translation *= 0.5;
          strafe *= 0.5;
          rotation *= 0.5;
        }

        double translationVal =
          translationLimiter.calculate(
            MathUtil.applyDeadband(
              translation, Constants.SwerveConstants.SWERVE_DEADBAND));
        double strafeVal =
          strafeLimiter.calculate(
            MathUtil.applyDeadband(
              strafe, Constants.SwerveConstants.SWERVE_DEADBAND));
        double rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(
                    rotation, Constants.SwerveConstants.SWERVE_DEADBAND));

        drive(
            new Translation2d(translationVal,strafeVal).times(SwerveConstants.MAX_SPEED_METERS_PER_SECOND),
            rotationVal * swerve.swerveController.config.maxAngularVelocity,
            true,
            false);
      })
      .withName("TeleopSwerve");
  }

  public void drive(
    Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop);
  }

  public Command driveHeading(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier headingXSup,
    DoubleSupplier headingYSup
  ) {
      return run(() -> {
      double translation = translationSup.getAsDouble();
      double strafe = strafeSup.getAsDouble();
      double headingX = headingXSup.getAsDouble();
      double headingY = headingYSup.getAsDouble();

      double translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(
                  translation, Constants.SwerveConstants.SWERVE_DEADBAND));
      double strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(
                  strafe, Constants.SwerveConstants.SWERVE_DEADBAND));

      ChassisSpeeds desiredSpeeds
          = swerve.swerveController.getTargetSpeeds(
              translationVal,
              strafeVal,
              headingX,
              headingY,
              swerve.getYaw().getRadians(),
              SwerveConstants.MAX_SPEED_METERS_PER_SECOND
          );

      driveHeading(desiredSpeeds);
    })
    .withName("TeleopHeadingSwerve");
  }

  public void driveHeading(ChassisSpeeds velocity)
  {
    swerve.driveFieldOriented(velocity); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public Command driveHeading(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier headingSup
  ) {
      return run(() -> {
      double translation = translationSup.getAsDouble();
      double strafe = strafeSup.getAsDouble();
      double angle = headingSup.getAsDouble();

      double translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(
                  translation, Constants.SwerveConstants.SWERVE_DEADBAND));
      double strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(
                  strafe, Constants.SwerveConstants.SWERVE_DEADBAND));

      ChassisSpeeds desiredSpeeds
          = swerve.swerveController.getTargetSpeeds(
              translationVal,
              strafeVal,
              angle,
              swerve.getYaw().getRadians(),
              SwerveConstants.MAX_SPEED_METERS_PER_SECOND
          );

      driveHeading(desiredSpeeds);
    })
    .withName("TeleopStickyHeadingSwerve");
  }

  public void driveStickyHeading(ChassisSpeeds velocity) {
    swerve.driveFieldOriented(velocity); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void lock() {
    swerve.lockPose();
  }

  public double getYaw() {
    return swerve.getYaw().getDegrees();
  }

  public double getPitch() {
    return swerve.getPitch().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerve.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    swerve.drive(targetSpeeds);
  }

  @Override
  public void periodic() {
    // // Update the robot odometry per scheduler run //
    // swerve.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
