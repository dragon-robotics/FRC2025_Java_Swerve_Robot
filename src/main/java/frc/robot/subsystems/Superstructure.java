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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
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

  public enum ReefAlignmentStates {
    ALIGN_REEF_LEFT_L1, // Default state
    ALIGN_REEF_LEFT_L2,
    ALIGN_REEF_LEFT_L3,
    ALIGN_REEF_LEFT_L4,
    ALIGN_REEF_RIGHT_L1,
    ALIGN_REEF_RIGHT_L2,
    ALIGN_REEF_RIGHT_L3,
    ALIGN_REEF_RIGHT_L4,
  }

  private ReefAlignmentStates m_reefAlignmentState;

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

  private boolean useLeftCamera;                          // Keeps track of the camera to use
  private boolean intakeCoralLeft;                        // Keeps track of the coral intake side
  private ElevatorSubsystem.ElevatorState elevatorState;  // Keeps track of the current elevator state
  private double elevatorHeightTranslationFactor;         // Keeps track of the elevator translation speed factor
  private double elevatorHeightStrafeFactor;              // Keeps track of the elevator strafe speed factor
  private double elevatorHeightRotationFactor;            // Keeps track of the elevator rotation speed factor

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
    m_reefAlignmentState = ReefAlignmentStates.ALIGN_REEF_LEFT_L1;

    // Instantiate current heading as empty //
    currentHeading = Optional.empty(); // Keeps track of current heading

    // Instantiate the rotation last triggered as 0 //
    rotationLastTriggered = 0.0;

    // Instantiate the elevator wanted state as home //
    elevatorState = ElevatorSubsystem.ElevatorState.HOME;

    // Default we use the right camera //
    useLeftCamera = false;

    // Initialize the elevator height speed factor //
    elevatorHeightTranslationFactor = 1.0;
    elevatorHeightStrafeFactor = 1.0;
    elevatorHeightRotationFactor = 1.0;

    // We intake coral from the left side by default //
    intakeCoralLeft = true;

    // Instantiate the logger for telemetry //
    logger = new Telemetry(maxSpeed);

    // Register the telemetry for the swerve drive //
    m_swerve.registerTelemetry(logger::telemeterize);
  }

  // Swerve Drive Subsystem Commands //

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
      MathUtil.applyDeadband(rawTranslation, SwerveConstants.SWERVE_DEADBAND, 0.1);
      double translation = MathUtil.applyDeadband(
          rawTranslation,
          SwerveConstants.SWERVE_DEADBAND,
          1); // Forward/Backward
      double strafe = MathUtil.applyDeadband(
          rawStrafe,
          SwerveConstants.SWERVE_DEADBAND,
          1); // Left/Right
      double rotation = MathUtil.applyDeadband(
          rawRotation,
          SwerveConstants.SWERVE_DEADBAND,
          1); // Rotation

      // Square the inputs (while preserving the sign) to increase fine control while
      // permitting full power //
      translation = Math.copySign(translation * translation, translation);
      strafe = Math.copySign(strafe * strafe, strafe);
      rotation = Math.copySign(rotation * rotation, rotation);

      // Multiply the translation, strafe, and rotation by the elevator height speed factor //
      translation *= elevatorHeightTranslationFactor;
      strafe *= elevatorHeightStrafeFactor;
      rotation *= elevatorHeightRotationFactor;

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

  public Command SwerveBrake() {
    return m_swerve.applyRequest(() -> brake);
  }

  public Command SeedFieldCentric() {
    return m_swerve.runOnce(() -> m_swerve.seedFieldCentric());
  }

  // Elevator Subsystem Commands //

  public Command ElevatorIdle() {
    Command resetEncoder = new InstantCommand(() -> {
      m_elevator.seedElevatorMotorEncoderPosition(0);
    });

    Command setElevatorToIdle = new InstantCommand(() -> {
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE);
    }, m_elevator);

    Command resetElevatorEncoder = new ConditionalCommand(
      resetEncoder.andThen(setElevatorToIdle),
      setElevatorToIdle,
      m_elevator::isAtElevatorBottom
    );

    return resetElevatorEncoder;
  }

  public Command ElevatorHome() {
        
    Command setElevatorHome = new InstantCommand(() -> {
      elevatorHeightTranslationFactor = 1.0;
      elevatorHeightStrafeFactor = 1.0;
      elevatorHeightRotationFactor = 1.0;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.HOME);
    }, m_elevator);

    Command waitUntilElevatorIsAtBottom = new WaitUntilCommand(() -> m_elevator.isAtElevatorBottom());

    Command reZeroElevatorEncoder = new InstantCommand(() -> {
      m_elevator.seedElevatorMotorEncoderPosition(0);
    });

    return setElevatorHome
          .andThen(waitUntilElevatorIsAtBottom)
          .andThen(reZeroElevatorEncoder);

    // If the elevator is at the home position, check if there is a current spike //
    // If there is a current spike, reset the elevator encoder, then set to idle //
    // else set the motor to idle
  }

  public Command ElevatorL1() {

    Command setElevatorL1 = new RunCommand(() -> {
      elevatorHeightTranslationFactor = 1.0;
      elevatorHeightStrafeFactor = 1.0;
      elevatorHeightRotationFactor = 1.0;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
    }, m_elevator);

    return setElevatorL1;
  }

  public Command ElevatorL2() {
    
    Command setElevatorL2 = new RunCommand(() -> {
      elevatorHeightTranslationFactor = 0.75;
      elevatorHeightStrafeFactor = 0.75;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L2);
    }, m_elevator);

    return setElevatorL2;
  }

  public Command ElevatorL3() {

    Command setElevatorL3 = new RunCommand(() -> {
      elevatorHeightTranslationFactor = 0.65;
      elevatorHeightStrafeFactor = 0.65;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L3);
    }, m_elevator);

    return setElevatorL3;
  }

  public Command ElevatorL4() {
    
    Command setElevatorL4 = new RunCommand(() -> {
      elevatorHeightTranslationFactor = 0.5;
      elevatorHeightStrafeFactor = 0.5;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L4);
    }, m_elevator);

    return setElevatorL4;
  }

  // Coral Subsystem Commands //
  public Command SetCoralStation(boolean left) {
    Command setCoralStation = new InstantCommand(() -> {
      intakeCoralLeft = left;
    });

    return setCoralStation;
  }

  public Command IntakeCoral(){

    Command setRobotHeading = new InstantCommand(() -> {
      // Set the robot heading to the coral station angle for intake //
      currentHeading = intakeCoralLeft ? 
        Optional.of(GeneralConstants.LEFT_CORAL_STATION_INTAKE_ANGLE) : // If left, set to left side angle
        Optional.of(GeneralConstants.RIGHT_CORAL_STATION_INTAKE_ANGLE); // If right, set to right side angle
    });

    Command engageCoralIntake = new InstantCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.INTAKE),
      m_coral
    );

    Command runUntilCoralIsDetected = new WaitUntilCommand(() -> m_coral.isBeamBreakTripped());

    Command slowIntake = new InstantCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOW_INTAKE),
      m_coral
    );

    Command runUntilCoralIsNotDetected = new WaitUntilCommand(() -> !m_coral.isBeamBreakTripped());

    // Run until the beambreak or current limit is tripped // 
    return setRobotHeading
    .andThen(engageCoralIntake)
    .andThen(runUntilCoralIsDetected)
    .andThen(slowIntake)
    .andThen(runUntilCoralIsNotDetected);
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

  public Command AlignToScoreCoral(){

    Command autoDriveToReefStation = AimAndRangeApriltag();

    Command setElevatorLevel = new InstantCommand(() -> {
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
    }, m_elevator);

    Command waitUntilElevatorIsAtLevel = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return autoDriveToReefStation
          .andThen(setElevatorLevel)
          .andThen(waitUntilElevatorIsAtLevel);
  }

  public Command ScoreCoral() {
    Command scoreCoral = new RunCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.EJECT),
      m_coral
    );

    return scoreCoral;
  }

  public Command HoldCoral() {
    Command holdCoral = new RunCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.HOLD),
      m_coral
    );

    return holdCoral;
  }

  // Algae Subsystem Commands //

  public Command IntakeAlgae(){

    Command engageAlgaeIntake = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE),
      m_algae
    );

    Command runUntilAlgaeIsDetected = new WaitUntilCommand(() -> m_algae.hasAlgae());

    Command holdAlgae = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD),
      m_algae
    );

    // Run until the beambreak or current limit is tripped // 
    return engageAlgaeIntake;
  }

  public Command DeAlgaeify() {
    Command engageAlgaeDeAlgaeify = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE),
      m_algae
    );

    return engageAlgaeDeAlgaeify;
  }

  public Command ScoreAlgae() {
    Command scoreAlgae = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.EJECT),
      m_algae
    );

    return scoreAlgae;
  }

  public Command AlgaeArmHome() {
    Command homeAlgaeArm = new RunCommand(() -> {
      m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOME);
    }, m_algae);

    return homeAlgaeArm;
  }

  public Command AlgaeArmIntake() {
    Command setAlgaeArmIntake = new RunCommand(() -> {
      m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE);
    }, m_algae);

    return setAlgaeArmIntake;
  }

  public Command AlgaeArmDeAlgaeify() {
    Command setAlgaeArmDeAlgaeify = new RunCommand(() -> {
      m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE);
    }, m_algae);

    return setAlgaeArmDeAlgaeify;
  }

  public Command AlgaeArmHold() {
    Command setAlgaeArmHold = new RunCommand(() -> {
      m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD);
    }, m_algae);

    return setAlgaeArmHold;
  }

  public Command AlgaeArmEject() {
    Command setAlgaeArmEject = new RunCommand(() -> {
      m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.EJECT);
    }, m_algae);

    return setAlgaeArmEject;
  }

  /** Robot state configurations - operator (2nd driver) sets up what the score button does */
  public void setReefAlignment(ReefAlignmentStates wantedReefAlignmentState) {
    switch(wantedReefAlignmentState){
      case ALIGN_REEF_LEFT_L1:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = true;                                     // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L2:
        elevatorState = ElevatorSubsystem.ElevatorState.L2; // Set the elevator to L2
        useLeftCamera = true;                                     // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L3:
        elevatorState = ElevatorSubsystem.ElevatorState.L3; // Set the elevator to L1
        useLeftCamera = true;                                     // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L4:
        elevatorState = ElevatorSubsystem.ElevatorState.L4; // Set the elevator to L1
        useLeftCamera = true;                                     // Use the left camera
        break;
      case ALIGN_REEF_RIGHT_L1:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = false;                                    // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L2:
        elevatorState = ElevatorSubsystem.ElevatorState.L2; // Set the elevator to L1
        useLeftCamera = false;                                    // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L3:
        elevatorState = ElevatorSubsystem.ElevatorState.L3; // Set the elevator to L3
        useLeftCamera = false;                                    // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L4:
        elevatorState = ElevatorSubsystem.ElevatorState.L4; // Set the elevator to L4
        useLeftCamera = false;                                    // Use the right camera
        break;
      default:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = true;                                     // Use the left camera
        break;
    }
  }

  public Command setReefAlignmentCommand(ReefAlignmentStates wantedReefAlignmentState) {
    return new InstantCommand(() -> setReefAlignment(wantedReefAlignmentState));
  }

  public void setReefAlignment(boolean left){
    useLeftCamera = left;
  }

  public Command SetReefAlignment(boolean left){
    return new InstantCommand(() -> setReefAlignment(left)); 
  }

  public void setReefHeight(ElevatorState wantedElevatorState){
    elevatorState = wantedElevatorState;
  }

  public Command SetReefHeight(ElevatorState wantedElevatorState){
    return new InstantCommand(() -> setReefHeight(wantedElevatorState));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
