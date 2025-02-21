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
    STOPPED,                        // Stopped state
    DEFAULT,                        // Default driving state
    DEFAULT_WITH_CORAL,             // Default driving state w/ coral
    DEFAULT_WITH_ALGAE,             // Default driving state w/ algae
    DEFAULT_WITH_CORAL_AND_ALGAE,   // Default driving state w/ coral and algae
    ALIGN_INTAKE_CORAL_LEFT,
    ALIGN_INTAKE_CORAL_RIGHT,
    INTAKE_CORAL,
    INTAKE_ALGAE,
    ALIGN_REEF_LEFT_L1,
    ALIGN_REEF_LEFT_L2,
    ALIGN_REEF_LEFT_L3,
    ALIGN_REEF_LEFT_L4,
    ALIGN_REEF_RIGHT_L1,
    ALIGN_REEF_RIGHT_L2,
    ALIGN_REEF_RIGHT_L3,
    ALIGN_REEF_RIGHT_L4,
    ALIGN_TO_SCORE_CORAL,
    SCORE_CORAL,
    SCORE_PROCESSOR
  }

  public enum CurrentSuperState {
    STOPPED,                        // Stopped state
    DEFAULT,                        // Default driving state
    DEFAULT_WITH_CORAL,             // Default driving state w/ coral
    DEFAULT_WITH_ALGAE,             // Default driving state w/ algae
    DEFAULT_WITH_CORAL_AND_ALGAE,   // Default driving state w/ coral and algae
    ALIGN_INTAKE_CORAL_LEFT,
    ALIGN_INTAKE_CORAL_RIGHT,
    INTAKE_CORAL,
    INTAKE_ALGAE,
    ALIGN_REEF_LEFT_L1,
    ALIGN_REEF_LEFT_L2,
    ALIGN_REEF_LEFT_L3,
    ALIGN_REEF_LEFT_L4,
    ALIGN_REEF_RIGHT_L1,
    ALIGN_REEF_RIGHT_L2,
    ALIGN_REEF_RIGHT_L3,
    ALIGN_REEF_RIGHT_L4,
    ALIGN_TO_SCORE_CORAL,
    SCORE_CORAL,
    SCORE_PROCESSOR
  }

  private WantedSuperState wantedSuperState;
  private CurrentSuperState currentSuperState;
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

  private double rotationLastTriggered; // Keeps track of the last time the rotation was triggered
  private Optional<Rotation2d> currentHeading; // Keeps track of current heading

  private boolean useLeftCamera;
  private boolean intakeCoralLeft;
  private ElevatorSubsystem.WantedState elevatorWantedState;

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
    visionRangePID.setTolerance(VisionConstants.RANGE_TOLERANCE);
    visionAimPID = new PIDController(VisionConstants.AIM_P, VisionConstants.AIM_I, VisionConstants.AIM_D);
    visionAimPID.setTolerance(VisionConstants.AIM_TOLERANCE);

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

    // Instantiate current and wanted super states as stopped //
    wantedSuperState = WantedSuperState.STOPPED;
    currentSuperState = CurrentSuperState.STOPPED;

    // Instantiate current heading as empty //
    currentHeading = Optional.empty(); // Keeps track of current heading

    // Instantiate the rotation last triggered as 0 //
    rotationLastTriggered = 0.0;

    // Instantiate the elevator wanted state as home //
    elevatorWantedState = ElevatorSubsystem.WantedState.HOME;

    // Default we use the right camera //
    useLeftCamera = false;

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

  public Command AimAndRangeApriltag() {
    return new RunCommand(() -> {
      // Read in relevant data from the Camera
      boolean targetVisible = false;
      double tagYaw = 0.0;
      double tagRange = 0.0;
      int bestTagId = 0;

      var results = m_vision.getCamera(useLeftCamera).getAllUnreadResults();
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
                    useLeftCamera ? APTAG_ALIGN_LEFT_CAM_POS.getZ() : APTAG_ALIGN_RIGHT_CAM_POS.getZ(), // Measured with a tape measure, or in CAD.
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
            " At Range Setpoint: " + Boolean.toString(visionRangePID.atSetpoint()) +
            " TagRange: " + Double.toString(tagRange) +
            " RangeError: " + Double.toString(rangeError) +
            " RangeCorrection: " + Double.toString(forwardCorrection) +
            " At Yaw Setpoint: " + Boolean.toString(visionAimPID.atSetpoint()) +
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
    }, m_vision, m_swerve) {
      @Override
      public void end(boolean interrupted) {
        // This runs whether the command finishes normally or is interrupted.
        m_swerve.setOperatorPerspectiveForward(Rotation2d.kZero);
        currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
      }
    }
    .until(() -> visionRangePID.atSetpoint() && visionAimPID.atSetpoint());
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
      case DEFAULT:
        if (m_coral.hasCoral() && m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL_AND_ALGAE;
        } else if (m_coral.hasCoral()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL;
        } else if (m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_ALGAE;
        } else {
          currentSuperState = CurrentSuperState.DEFAULT;
        }
        break;
      case DEFAULT_WITH_CORAL:
        currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL;
        break;
      case DEFAULT_WITH_ALGAE:
        currentSuperState = CurrentSuperState.DEFAULT_WITH_ALGAE;
        break;
      case DEFAULT_WITH_CORAL_AND_ALGAE:
        currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL_AND_ALGAE;
        break;
      case INTAKE_CORAL:
        if (m_coral.hasCoral() && m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL_AND_ALGAE;
        } else if (m_coral.hasCoral()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL;
        } else {
          currentSuperState = CurrentSuperState.INTAKE_CORAL;
        }
        break;
      case INTAKE_ALGAE:
        if (m_coral.hasCoral() && m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_CORAL_AND_ALGAE;
        } else if (m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.DEFAULT_WITH_ALGAE;
        } else {
          currentSuperState = CurrentSuperState.INTAKE_ALGAE;
        }
        break;
      case SCORE_CORAL:
        // If the robot has no coral, else do nothing //
        if (m_coral.hasCoral()) {
          currentSuperState = CurrentSuperState.SCORE_CORAL;
        }
        break;
      case SCORE_PROCESSOR:
        // If the robot has no algae, else do nothing //
        if (m_algae.hasAlgae()) {
          currentSuperState = CurrentSuperState.SCORE_PROCESSOR;
        }
        break;
      case ALIGN_TO_SCORE_CORAL:
        currentSuperState = CurrentSuperState.ALIGN_TO_SCORE_CORAL;
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
      case DEFAULT:
        setDefault(false, false);
        break;
      case DEFAULT_WITH_CORAL:
        setDefault(true, false);
        break;
      case DEFAULT_WITH_ALGAE:
        setDefault(false, true);
        break;
      case DEFAULT_WITH_CORAL_AND_ALGAE:
        setDefault(true, true);
        break;
      case INTAKE_CORAL:
        intakeCoral(intakeCoralLeft);
        break;
      case INTAKE_ALGAE:
        intakeAlgae();
        break;
      case ALIGN_TO_SCORE_CORAL:
        alignToScoreCoral();
        break;
      case SCORE_CORAL:
        // If there is a coral in the robot, score it, otherwise, do nothing //
        if (m_coral.hasCoral()) {
          m_coral.setWantedState(CoralSubsystem.WantedState.EJECT);
        }
        break;
      case SCORE_PROCESSOR:
        scoreAlgae();
        break;
      case STOPPED:
      default:
        stopped();
        break;
    }
  }

  private void setDefault(boolean hasCoral, boolean hasAlgae) {
    if(hasCoral && hasAlgae){
      // Default state with coral and algae //
      m_coral.setWantedState(CoralSubsystem.WantedState.HOLD);
      m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOLD);
    } else if (hasCoral && !hasAlgae){
      // Default state with coral //
      m_coral.setWantedState(CoralSubsystem.WantedState.HOLD);
      m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOME);
    } else if (!hasCoral && hasAlgae){
      // Default state with algae //
      m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
      m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOLD);
    } else {
      // Default state without coral or algae on robot //
      m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
      m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOME);
    }
  }

  private void intakeCoral(boolean left) {

    // Set coral to intake //
    m_coral.setWantedState(CoralSubsystem.WantedState.INTAKE);
    // Set elevator to home //
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);

    // If the algae is already in the robot, hold the algae, otherwise, set algae to home //
    if (m_algae.hasAlgae()) {
      // Set algae to hold //
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOLD);
    } else {
      // Set algae to home //
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOME);
    }

    // Set drivetrain to the left coral station angle for intake //
    currentHeading = left ?
        Optional.of(GeneralConstants.LEFT_CORAL_STATION_INTAKE_ANGLE) : // If left, set to left side angle
        Optional.of(GeneralConstants.RIGHT_CORAL_STATION_INTAKE_ANGLE); // If right, set to right side angle
  }

  private void intakeAlgae() {

    // If the coral is already in the robot, hold the coral, otherwise, set coral to home //
    if (m_coral.hasCoral()) {
      // Set coral to hold //
      m_coral.setWantedState(CoralSubsystem.WantedState.HOLD);
    } else {
      // Set coral to idle //
      m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
    }

    // Set elevator to home //
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
    // Set algae to intake //
    m_algae.setWantedState(AlgaeSubsystem.WantedState.INTAKE);
  }

  private void alignToScoreCoral() {
    // If the algae is already in the robot, hold the algae, otherwise, set algae to home //
    if (m_algae.hasAlgae()) {
      // Set algae to hold //
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOLD);
    } else {
      // Set algae to home //
      m_algae.setWantedState(AlgaeSubsystem.WantedState.HOME);
    }

    // Set the coral to hold //
    m_coral.setWantedState(CoralSubsystem.WantedState.HOLD);

    // Set the elevator to the wanted state //
    m_elevator.setWantedState(elevatorWantedState);
  }

  private void scoreAlgae() {
    // If the coral is already in the robot, hold the coral, otherwise, set coral to idle //
    if (m_coral.hasCoral()) {
      // Set coral to hold //
      m_coral.setWantedState(CoralSubsystem.WantedState.HOLD);
    } else {
      // Set coral to idle //
      m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
    }
    // Set the elevator to home //
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.HOME);
    // Set the algae to eject //
    m_algae.setWantedState(AlgaeSubsystem.WantedState.EJECT);
  }
  
  private void stopped() {
    // Set the coral to idle //
    m_coral.setWantedState(CoralSubsystem.WantedState.IDLE);
    // Set the elevator to idle //
    m_elevator.setWantedState(ElevatorSubsystem.WantedState.IDLE);
    // Set the algae to idle //
    m_algae.setWantedState(AlgaeSubsystem.WantedState.IDLE);
    // Set the swerve drive to stop //
    m_swerve.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  /** State pushers */
  public void setWantedSuperState(WantedSuperState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
  }

  /** Robot state configurations - operator (2nd driver) sets up what the score button does */
  public void setRobotState(WantedSuperState wantedSuperState) {
    switch(wantedSuperState){
      case ALIGN_REEF_LEFT_L1:
        elevatorWantedState = ElevatorSubsystem.WantedState.L1; // Set the elevator to L1
        useLeftCamera = true;                                   // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L2:
        elevatorWantedState = ElevatorSubsystem.WantedState.L2; // Set the elevator to L2
        useLeftCamera = true;                                   // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L3:
        elevatorWantedState = ElevatorSubsystem.WantedState.L3; // Set the elevator to L1
        useLeftCamera = true;                                   // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L4:
        elevatorWantedState = ElevatorSubsystem.WantedState.L4; // Set the elevator to L1
        useLeftCamera = true;                                   // Use the left camera
        break;
      case ALIGN_REEF_RIGHT_L1:
        elevatorWantedState = ElevatorSubsystem.WantedState.L1; // Set the elevator to L1
        useLeftCamera = false;                                  // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L2:
        elevatorWantedState = ElevatorSubsystem.WantedState.L2; // Set the elevator to L1
        useLeftCamera = false;                                  // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L3:
        elevatorWantedState = ElevatorSubsystem.WantedState.L3; // Set the elevator to L3
        useLeftCamera = false;                                  // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L4:
        elevatorWantedState = ElevatorSubsystem.WantedState.L4; // Set the elevator to L4
        useLeftCamera = false;                                  // Use the right camera
        break;
      case ALIGN_INTAKE_CORAL_LEFT:
        intakeCoralLeft = true;
        break;
      case ALIGN_INTAKE_CORAL_RIGHT:
        intakeCoralLeft = false;
        break;        
      default:
        break;
    }
  }

  public Command setRobotStateCommand(WantedSuperState wantedSuperState) {
    return new InstantCommand(() -> setRobotState(wantedSuperState));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}
