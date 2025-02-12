// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.VisionConstants.APTAG_ALIGN_LEFT_CAM_POS;
import static frc.robot.Constants.VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS;
import static frc.robot.Constants.VisionConstants.DESIRED_RANGE;
import static frc.robot.Constants.VisionConstants.DESIRED_YAW;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.swerve_constant.TunerConstants;

public class Superstructure extends SubsystemBase {

  private final CommandSwerveDrivetrain m_swerve;
  private final CoralSubsystem m_coral;
  private final ElevatorSubsystem m_elevator;
  private final AlgaeSubsystem m_algae;
  private final VisionSubsystem m_vision;
  private final RobotContainer m_container;
  private final Telemetry logger;

  public enum WantedSuperState {
    STOPPED, // Stopped state
    GENERAL, // Default / General driving state
    INTAKE_CORAL_LEFT,
    INTAKE_CORAL_RIGHT,
    INTAKE_ALGAE,
    ALGAE_STOWED,
    ALIGN_TO_REEF_LEFT,
    ALIGN_TO_REEF_RIGHT,
    ALIGN_TO_REEF_CENTER,
    HOME,
    L1,
    L2,
    L3,
    L4,
    SCORE_CORAL_TAG_6,
    SCORE_CORAL_TAG_7,
    SCORE_CORAL_TAG_8,
    SCORE_CORAL_TAG_9,
    SCORE_CORAL_TAG_10,
    SCORE_CORAL_TAG_11,
    SCORE_CORAL_TAG_17,
    SCORE_CORAL_TAG_18,
    SCORE_CORAL_TAG_19,
    SCORE_CORAL_TAG_20,
    SCORE_CORAL_TAG_21,
    SCORE_CORAL_TAG_22,
    SCORE_PROCESSOR
  }

  public enum CurrentSuperState {
    STOPPED, // Default state
    GENERAL, // General driving state
    INTAKE_CORAL_LEFT,
    INTAKE_CORAL_RIGHT,
    INTAKE_ALGAE,
    ALGAE_STOWED,
    ALIGN_TO_REEF_LEFT,
    ALIGN_TO_REEF_RIGHT,
    HOME,
    L1,
    L2,
    L3,
    L4,
    SCORE_CORAL_TAG_6,
    SCORE_CORAL_TAG_7,
    SCORE_CORAL_TAG_8,
    SCORE_CORAL_TAG_9,
    SCORE_CORAL_TAG_10,
    SCORE_CORAL_TAG_11,
    SCORE_CORAL_TAG_17,
    SCORE_CORAL_TAG_18,
    SCORE_CORAL_TAG_19,
    SCORE_CORAL_TAG_20,
    SCORE_CORAL_TAG_21,
    SCORE_CORAL_TAG_22,
    SCORE_PROCESSOR
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.RobotCentric driveRobotCentric;
  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  private final double maxSpeed;
  private final double maxAngularRate;
  private PIDController visionRangePID;
  private PIDController visionAimPID;

  private double rotationLastTriggered = 0.0; // Keeps track of the last time the rotation was triggered
  private Optional<Rotation2d> currentHeading = Optional.empty(); // Keeps track of current heading

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      CoralSubsystem coral,
      ElevatorSubsystem elevator,
      AlgaeSubsystem algae,
      VisionSubsystem vision,
      RobotContainer container) {
    m_swerve = swerve;
    m_coral = coral;
    m_elevator = elevator;
    m_algae = algae;
    m_vision = vision;
    m_container = container;

    // Instatiate swerve max speed and angular rate //
    maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
    maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    // Instantiate vision PID controllers //
    visionRangePID = new PIDController(VisionConstants.RANGE_P, VisionConstants.RANGE_I, VisionConstants.RANGE_D);
    visionAimPID = new PIDController(VisionConstants.AIM_P, VisionConstants.AIM_I, VisionConstants.AIM_D);

    // Instantiate default field centric drive (no need to maintain heading) //
    drive = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * 0.05)
        .withRotationalDeadband(maxAngularRate * 0.05) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate brake (X-lock swerve wheels) //
    brake = new SwerveRequest.SwerveDriveBrake();

    // Instantiate point (point swerve wheels in a specific direction) //
    point = new SwerveRequest.PointWheelsAt();

    // Instantiate robot centric drive (forward is based on forward pose of the
    // robot) //
    driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Instantiate field centric drive (maintain heading) //
    driveMaintainHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(maxSpeed * 0.05)
        .withRotationalDeadband(maxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withDesaturateWheelSpeeds(true);

    // Set the PID constants for the Maintain Heading controller //
    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP,
        SwerveConstants.HEADING_KI,
        SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Instantiate the logger for telemetry //
    logger = new Telemetry(maxSpeed);

    // Register the telemetry for the swerve drive //
    m_swerve.registerTelemetry(logger::telemeterize);
  }

  public Command DefaultDriveCommand(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup) {
    return new RunCommand(() -> {

      double rawTranslation = translationSup.getAsDouble();
      double rawStrafe = strafeSup.getAsDouble();
      double rawRotation = rotationSup.getAsDouble();

      // Apply deadband to joystick inputs //
      double translation = MathUtil.applyDeadband(
          rawTranslation,
          SwerveConstants.SWERVE_DEADBAND); // Forward/Backward
      double strafe = MathUtil.applyDeadband(
          rawStrafe,
          SwerveConstants.SWERVE_DEADBAND); // Left/Right
      double rotation = MathUtil.applyDeadband(
          rawRotation,
          SwerveConstants.SWERVE_DEADBAND); // Rotation

      double magnitude = Math.hypot(translation, strafe);

      if (magnitude < SwerveConstants.SWERVE_DEADBAND) {
        translation = 0;
        strafe = 0;
      } else {
        // Square the inputs (while preserving the sign) to increase fine control while
        // permitting full power //
        translation = Math.copySign(translation * translation, translation);
        strafe = Math.copySign(strafe * strafe, strafe);
        rotation = Math.copySign(rotation * rotation, rotation);
      }

      // Scale the output by half if half speed is enabled //
      if (halfSpeedSup.getAsBoolean()) {
        translation *= 0.5;
        strafe *= 0.5;
        rotation *= 0.5;
      }

      // Translate input into velocities for the swerve drive //
      translation *= maxSpeed; // Forward/Backward
      strafe *= maxSpeed; // Left/Right
      rotation *= maxAngularRate; // Rotation

      boolean rotationTriggered = Math.abs(rawRotation) > SwerveConstants.SWERVE_DEADBAND;
      boolean rotationActive = MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1) &&
          (Math.abs(m_swerve.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(10));

      // Check if the rotation of the joystick has been triggered //
      if (rotationTriggered) {
        // Timestamp the last time the rotation was triggered //
        rotationLastTriggered = Timer.getFPGATimestamp();
      }

      if (rotationTriggered || rotationActive) {
        // If the rotation is triggered or active, set the swerve drive to rotate in
        // place //
        m_swerve.setControl(
            drive
                .withVelocityX(translation)
                .withVelocityY(strafe)
                .withRotationalRate(rotation));
        currentHeading = Optional.empty();
      } else {
        // Initialize the current heading if it is empty
        if (currentHeading.isEmpty()) {
          currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
        }

        m_swerve.setControl(
            driveMaintainHeading
                .withVelocityX(translation)
                .withVelocityY(strafe)
                .withTargetDirection(currentHeading.get()));
      }

    }, m_swerve);
  }

  public Command AimAndRangeApriltag(
    BooleanSupplier isLeft
  ) {
    return new RunCommand(() -> {
      // Read in relevant data from the Camera
      boolean targetVisible = false;
      boolean isLeftCamera = isLeft.getAsBoolean();
      double tagYaw = 0.0;
      double tagRange = 0.0;
      int bestTagId = 0;

      var results = m_vision.getCamera(isLeft.getAsBoolean()).getAllUnreadResults();
      if (!results.isEmpty()) {
          // Camera processed a new frame since last
          // Get the last one in the list.
          var result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              // Get the best target
              var bestTag = result.getBestTarget();
              bestTagId = bestTag.getFiducialId();

              if (Arrays.stream(GeneralConstants.REEF_STATION_TAG_IDS).anyMatch(i -> i == bestTag.getFiducialId())) {
                  // Found a red station tag, record its information
                  tagYaw = bestTag.getYaw();
                  bestTag.getSkew();
                  tagRange =
                          PhotonUtils.calculateDistanceToTargetMeters(
                                  isLeftCamera ? APTAG_ALIGN_LEFT_CAM_POS.getZ() : APTAG_ALIGN_RIGHT_CAM_POS.getZ(), // Measured with a tape measure, or in CAD.
                                  0.308, // From 2025 game manual for red station tags
                                  Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                                  Units.degreesToRadians(bestTag.getPitch()));

                  targetVisible = true;
              }
          }
      }

      // If the target is visible, aim and range to it
      if (targetVisible) {
        // Override the driver's turn and fwd/rev command with an automatic one
        // That turns strafe towards the tag, and gets the range right.

        // Calculate range error
        double rangeError = tagRange - DESIRED_RANGE; // 8.364 inches is the distance to the wall of the reef
        double forwardCorrection = visionRangePID.calculate(tagRange, DESIRED_RANGE);

        // Calculate yaw error
        double yawError = tagYaw - DESIRED_YAW;
        double strafeCorrection = visionAimPID.calculate(tagYaw, DESIRED_YAW);

        double forward = -forwardCorrection;
        double strafe = strafeCorrection;

        // Optionally clamp outputs to your robot’s maximum speed.
        forward = MathUtil.clamp(forward, -maxSpeed, maxSpeed);
        strafe  = MathUtil.clamp(strafe, -maxSpeed, maxSpeed);

        System.out.println(
            "Strafe: " + Double.toString(strafe) +
            " Forward: " + Double.toString(forward) +
            " TagRange: " + Double.toString(tagRange) +
            " RangeError: " + Double.toString(rangeError) +
            " RangeCorrection: " + Double.toString(forwardCorrection) +
            " TagYaw: " + Double.toString(tagYaw) +
            " YawError: " + Double.toString(yawError) +
            " YawCorrection: " + Double.toString(strafeCorrection));

        m_swerve.setOperatorPerspectiveForward(GeneralConstants.REEF_STATION_ID_ANGLE_MAP.get(bestTagId));
        m_swerve.setControl(
            driveMaintainHeading
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withTargetDirection(Rotation2d.kZero));
      }
    }, m_vision, m_swerve);
  }

  public Command SwerveBrake() {
    return m_swerve.applyRequest(() -> brake);
  }

  public Command SeedFieldCentric() {
    return m_swerve.runOnce(() -> m_swerve.seedFieldCentric());
  }

  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case INTAKE_CORAL_LEFT:
        currentSuperState = CurrentSuperState.INTAKE_CORAL_LEFT;
        break;
      case INTAKE_CORAL_RIGHT:
        currentSuperState = CurrentSuperState.INTAKE_CORAL_RIGHT;
        break;
      case STOPPED:
      default:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case INTAKE_CORAL_LEFT:
        intakeCoral(true);
        break;
      case INTAKE_CORAL_RIGHT:
        intakeCoral(false);
        break;
      case GENERAL:
        setDefault();
        break;
      case STOPPED:
      default:
        stopped();
        break;
    }
  }

  private void stopped() {
    m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.IDLE);
    m_algae.setWantedState(AlgaeSubsystem.WantedState.IDLE);
    m_swerve.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  private void setDefault(){
    m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.IDLE);
    m_algae.setWantedState(AlgaeSubsystem.WantedState.IDLE);
  }

  private void intakeCoral(boolean left) {

    // Set coral to intake //
    m_coral.setWantedState(CoralSubsystem.WantedState.INTAKE);
    // Set elevator to idle //
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.IDLE);

    // Set drivetrain to the left coral station angle for intake //
    currentHeading = left ?
        Optional.of(GeneralConstants.LEFT_CORAL_STATION_INTAKE_ANGLE) : // If left, set to left side angle
        Optional.of(GeneralConstants.RIGHT_CORAL_STATION_INTAKE_ANGLE); // If right, set to right side angle

    // // Set the swerve drive to maintain the heading of the coral station angle //
    // m_swerve.setControl(
    //     driveMaintainHeading
    //         .withTargetDirection(intakeHeading));
  }

  /** State pushers */
  public void setWantedSuperState(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}
