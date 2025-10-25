// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Teleop.DriveToPosePID;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.controller.ControllerSubsystem;
import frc.robot.subsystems.controller.ControllerSubsystem.ControllerState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.RobotContainer;
import frc.robot.swerve_constant.TunerConstants;
import frc.robot.util.Telemetry;

public class Superstructure extends SubsystemBase {

  private final CommandSwerveDrivetrain m_swerve;
  private final CoralSubsystem m_coral;
  private final ElevatorSubsystem m_elevator;
  private final AlgaeSubsystem m_algae;
  private final VisionSubsystem m_vision;
  private final ControllerSubsystem m_controller;
  private final RobotContainer m_container;
  private final Telemetry logger;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final SwerveRequest.RobotCentric driveRobotCentric;
  private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  private final double maxSpeed;
  private final double maxAngularRate;
  private PIDController visionRangePID;
  private PIDController visionAimPID;

  private double rotationLastTriggered; // Keeps track of the last time the rotation was triggered
  private Optional<Rotation2d> currentHeading; // Keeps track of current heading

  private double elevatorHeightTranslationFactor;         // Keeps track of the elevator translation speed factor
  private double elevatorHeightStrafeFactor;              // Keeps track of the elevator strafe speed factor
  private double elevatorHeightRotationFactor;            // Keeps track of the elevator rotation speed factor

  // Cache the closest reef pose, updated periodically
  private Pose2d cachedClosestLeftReef = new Pose2d();
  private Pose2d cachedClosestRightReef = new Pose2d();
  private Pose2d cachedClosestCoralStation = new Pose2d();

  private int updateCounter = 0;
  private static final int CACHED_CLOSEST_POSE_UPDATE_RATE =
      3; // Update every 3rd call to periodic()

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      CoralSubsystem coral,
      ElevatorSubsystem elevator,
      AlgaeSubsystem algae,
      VisionSubsystem vision,
      ControllerSubsystem controller,
      RobotContainer container) {
    m_swerve = swerve;
    m_coral = coral;
    m_elevator = elevator;
    m_algae = algae;
    m_vision = vision;
    m_controller = controller;
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

    // Instantiate the Field and Robot Speeds Swerve Requests //
    applyFieldSpeeds
      = new SwerveRequest.ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds
      = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(true)
        .withDriveRequestType(DriveRequestType.Velocity);

    // Instantiate current heading as empty //
    currentHeading = Optional.empty(); // Keeps track of current heading

    // Instantiate the rotation last triggered as 0 //
    rotationLastTriggered = 0.0;

    // Initialize the elevator height speed factor //
    elevatorHeightTranslationFactor = 1.0;
    elevatorHeightStrafeFactor = 1.0;
    elevatorHeightRotationFactor = 1.0;

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
        translation *= 0.35;
        strafe *= 0.35;
        rotation *= 0.35;
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

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            m_swerve.setControl(
                driveMaintainHeading
                    .withVelocityX(translation)
                    .withVelocityY(strafe)
                    .withTargetDirection(
                      alliance.get() == Alliance.Blue ?
                        currentHeading.get() :
                        currentHeading.get().rotateBy(Rotation2d.fromDegrees(180))));
        }
      }

    }, m_swerve);
  }

  public Command DriveToClosestReefPoseCommand(boolean left) {
    return new DeferredCommand(
            () -> {
              Pose2d target = left ? cachedClosestLeftReef : cachedClosestRightReef;
              Transform2d offset = new Transform2d(-0.25, 0.0, Rotation2d.kZero);

              return new DriveToPosePID(m_swerve, applyRobotSpeeds, target.transformBy(offset))
                  .andThen(new DriveToPosePID(m_swerve, applyRobotSpeeds, target));
            },
            Set.of(m_swerve))
        .andThen(
            Commands.deadline(
                Commands.sequence(
                  new RunCommand(() -> m_controller.setControllerState(ControllerState.STRONG_RUMBLE), m_controller).withTimeout(0.5),
                  new RunCommand(() -> m_controller.setControllerState(ControllerState.NO_RUMBLE), m_controller).withTimeout(0.02)
                ),
                new InstantCommand(
                    () -> currentHeading = Optional.of(m_swerve.getState().Pose.getRotation()))
            )
        )
        .handleInterrupt(() -> {
          currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
          m_controller.setControllerState(ControllerState.NO_RUMBLE);
        });
  }
  
  public Command DriveToClosestCoralStationPoseCommand() {

    return new DeferredCommand(
        () -> new DriveToPosePID(m_swerve, applyRobotSpeeds, cachedClosestCoralStation),
        Set.of(m_swerve))
        .andThen(
            Commands.deadline(
                Commands.sequence(
                  new RunCommand(() -> m_controller.setControllerState(ControllerState.STRONG_RUMBLE), m_controller).withTimeout(0.5),
                  new RunCommand(() -> m_controller.setControllerState(ControllerState.NO_RUMBLE), m_controller).withTimeout(0.02)
                ),
                new InstantCommand(
                    () -> currentHeading = Optional.of(m_swerve.getState().Pose.getRotation()))
            )
        )
        .handleInterrupt(() -> {
          currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
          m_controller.setControllerState(ControllerState.NO_RUMBLE);
        });
  }

  // Helper method to find closest pose from a list
  private Pose2d findClosestPose(Pose2d currentPose, List<Pose2d> targetPoses) {
    Pose2d closestPose = null;
    double minDistance = Double.MAX_VALUE;
    
    for (Pose2d targetPose : targetPoses) {
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = targetPose;
      }
    }
    
    return closestPose;
  }  

  public Command DriveToPoseCommand(boolean usePID) {

    // Calculate the closest reef pose to the current pose //
    Pose2d closestPose = null;

    // Are we red or blue?
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we are red, use the red reef poses //
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      // Get the current robot pose //
      Pose2d currentPose = m_swerve.getState().Pose;

      double minDistanceSq = Double.MAX_VALUE; // Use squared distance to avoid sqrt

      // Iterate through the list of target poses
      for (Pose2d targetPose : FieldConstants.Reef.RED_REEF_STATION_POSES) {

        Transform2d translationDelta = targetPose.minus(currentPose);

        // Calculate the squared distance between the translations
        double distanceSq = translationDelta.getTranslation().getNorm();

        // If this pose is closer than the current minimum, update
        if (distanceSq < minDistanceSq) {
          minDistanceSq = distanceSq;
          closestPose = targetPose;
        }
      }
    }
    // If we are blue, use the blue reef poses //
    else {
      // Get the current robot pose //
      Pose2d currentPose = m_swerve.getState().Pose;

      double minDistanceSq = Double.MAX_VALUE; // Use squared distance to avoid sqrt

      // Iterate through the list of target poses
      for (Pose2d targetPose : FieldConstants.Reef.BLUE_REEF_STATION_POSES) {
        Transform2d translationDelta = targetPose.minus(currentPose);

        // Calculate the squared distance between the translations
        double distanceSq = translationDelta.getTranslation().getNorm();

        // If this pose is closer than the current minimum, update
        if (distanceSq < minDistanceSq) {
          minDistanceSq = distanceSq;
          closestPose = targetPose;
        }
      }
    }

    if (usePID) {
      return new DriveToPosePID(m_swerve, applyRobotSpeeds, closestPose);
    } else {
      // Use default constraints defined elsewhere (e.g., AutoConstants)
      PathConstraints constraints = new PathConstraints(
          5,
          3,
          Units.degreesToRadians(540),
          Units.degreesToRadians(720)
      );

      return AutoBuilder.pathfindToPose(
          closestPose,
          constraints
      );
    }
  }

  public void initCurrentHeading() {
    currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
  }

  public Command SwerveBrake() {
    return m_swerve.applyRequest(() -> brake);
  }

  public Command SeedFieldCentric() {
    return m_swerve.runOnce(() -> m_swerve.seedFieldCentric());
  }

  // Elevator Subsystem Commands //

  public Command ElevatorZero() {
    Command resetEncoder = new InstantCommand(() -> {
      m_elevator.seedElevatorMotorEncoderPosition(0);
    });

    return resetEncoder;
  }

  public Command ElevatorStop() {
    Command setElevatorToIdle = new InstantCommand(() -> {
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE);
    }, m_elevator);

    return setElevatorToIdle;
  }

  public Command ElevatorManualDown() {
    Command manuallyMoveElevatorDown = new RunCommand(() -> {
      m_elevator.setElevatorMotorSpeed(-0.5);
    }, m_elevator);

    return manuallyMoveElevatorDown;
  }

  public Command ElevatorManualUp() {
    Command manuallyMoveElevatorUp = new RunCommand(() -> {
      m_elevator.setElevatorMotorSpeed(0.5);
    }, m_elevator);

    return manuallyMoveElevatorUp;
  }

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
      m_elevator::isAtElevatorState
    );

    return resetElevatorEncoder;
  }

  public Command ElevatorHome() {
        
    Command setElevatorHome =
        new InstantCommand(
            () -> {
              elevatorHeightTranslationFactor = 1.0;
              elevatorHeightStrafeFactor = 1.0;
              elevatorHeightRotationFactor = 1.0;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.HOME);
            },
            m_elevator);

    Command waitUntilElevatorIsAtHome = new WaitUntilCommand(m_elevator::isAtElevatorState);

    Command setElevatorToIdle =
        new InstantCommand(
            () -> m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE), m_elevator);

    Command reZeroElevatorEncoder =
        new InstantCommand(() -> m_elevator.seedElevatorMotorEncoderPosition(0), m_elevator);

    if (m_elevator.isCurrentLimitTripped()) {
      // If the elevator is at the home position, check if there is a current spike //
      // If there is a current spike, rezero the encoder and set the elevator to idle //
      return setElevatorHome
          .andThen(waitUntilElevatorIsAtHome)
          .andThen(reZeroElevatorEncoder)
          .andThen(setElevatorToIdle);
    } else {
      return setElevatorHome
          .andThen(waitUntilElevatorIsAtHome)
          .andThen(
              setElevatorToIdle);
    }
  }

  public Command ElevatorL1() {

    Command setElevatorL1 = new InstantCommand(() -> {
      elevatorHeightTranslationFactor = 1.0;
      elevatorHeightStrafeFactor = 1.0;
      elevatorHeightRotationFactor = 1.0;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
    }, m_elevator);

    Command waitUntilElevatorIsAtL1 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL1
          .andThen(waitUntilElevatorIsAtL1);
  }

  public Command ElevatorL2() {
    
    Command setElevatorL2 = new InstantCommand(() -> {
      elevatorHeightTranslationFactor = 0.75;
      elevatorHeightStrafeFactor = 0.75;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L2);
    }, m_elevator);

    Command waitUntilElevatorIsAtL2 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL2
          .andThen(waitUntilElevatorIsAtL2);
  }

  public Command ElevatorL3() {

    Command setElevatorL3 = new InstantCommand(() -> {
      elevatorHeightTranslationFactor = 0.65;
      elevatorHeightStrafeFactor = 0.65;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L3);
    }, m_elevator);

    Command waitUntilElevatorIsAtL3 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL3
          .andThen(waitUntilElevatorIsAtL3);
  }

  public Command ElevatorL4() {
    
    Command setElevatorL4 = new InstantCommand(() -> {
      elevatorHeightTranslationFactor = 0.5;
      elevatorHeightStrafeFactor = 0.5;
      elevatorHeightRotationFactor = 1;
      m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L4);
    }, m_elevator);

    Command waitUntilElevatorIsAtL4 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL4
          .andThen(waitUntilElevatorIsAtL4);

  }

  // Coral Subsystem Commands //

  public Command IntakeCoral(){

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

    Command slowReverseIntake = new InstantCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE),
      m_coral
    );

    Command runUntilCoralIsDetectedAgain = new WaitUntilCommand(() -> m_coral.isBeamBreakTripped());

    Command slowIntakeAgain = new InstantCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOWER_INTAKE),
      m_coral
    );

    // Run until the beambreak or current limit is tripped // 
    return engageCoralIntake
    .andThen(runUntilCoralIsDetected)
    .andThen(slowIntake)
    .andThen(runUntilCoralIsNotDetected)
    .andThen(slowReverseIntake)
    .andThen(runUntilCoralIsDetectedAgain)
    .andThen(slowIntakeAgain)
    .andThen(new WaitCommand(0.02));
  }

  public Command ReverseCoralIntake() {
    Command slowReverseIntake = new RunCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE),
      m_coral
    );

    return slowReverseIntake;
  }

  public Command ScoreCoral() {
    Command scoreCoral = new RunCommand(
      () -> m_coral.setCoralState(CoralSubsystem.CoralState.SCORE),
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
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.SCORE),
      m_algae
    );

    return scoreAlgae;
  }

  public Command AlgaeArmHome() {
    Command homeAlgaeArm = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOME),
      m_algae
    );

    return homeAlgaeArm;
  }

  public Command AlgaeArmIntake() {
    Command setAlgaeArmIntake = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE),
      m_algae
    );

    return setAlgaeArmIntake;
  }

  public Command AlgaeArmDeAlgaeify() {
    Command setAlgaeArmDeAlgaeify = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE),
      m_algae
    );

    return setAlgaeArmDeAlgaeify;
  }

  public Command AlgaeArmHold() {
    Command setAlgaeArmHold = new RunCommand(
      () -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD),
      m_algae
    );

    return setAlgaeArmHold;
  }

  private void updateCachedClosestPoses() {
    Pose2d currentPose = m_swerve.getState().Pose;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Blue); // Default to Blue if no alliance
    }

    Alliance currentAlliance = alliance.get();
    boolean isRed = currentAlliance == Alliance.Red;

    // boolean shouldFlip = currentPose.getX() > FieldConstants.REEF_CENTER_X;

    List<Pose2d> leftReefPoses =
        isRed
            ? FieldConstants.Reef.RED_REEF_STATION_LEFT_POSES
            : FieldConstants.Reef.BLUE_REEF_STATION_LEFT_POSES;
    List<Pose2d> rightReefPoses =
        isRed
            ? FieldConstants.Reef.RED_REEF_STATION_RIGHT_POSES
            : FieldConstants.Reef.BLUE_REEF_STATION_RIGHT_POSES;
    List<Pose2d> coralStationPoses =
        isRed
            ? FieldConstants.CoralStation.RED_CORAL_STATION_POSES
            : FieldConstants.CoralStation.BLUE_CORAL_STATION_POSES;

    cachedClosestLeftReef = findClosestPose(currentPose, leftReefPoses);
    cachedClosestRightReef = findClosestPose(currentPose, rightReefPoses);
    cachedClosestCoralStation = findClosestPose(currentPose, coralStationPoses);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++updateCounter >= CACHED_CLOSEST_POSE_UPDATE_RATE) {
      updateCounter = 0;
      updateCachedClosestPoses();
    }
  }
}
