// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Teleop.DriveToPosePID;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;
import frc.robot.util.Telemetry;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {

  private final CommandSwerveDrivetrain swerve;
  private final CoralSubsystem coral;
  private final ElevatorSubsystem elevator;
  private final AlgaeSubsystem algae;
  private final VisionSubsystem vision;
  private final RobotContainer container;
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

  private ReefAlignmentStates reefAlignmentState;

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

  private ElevatorSubsystem.ElevatorState elevState; // Keeps track of the current elevator state
  private double elevHeightTranslationFactor; // Keeps track of the elevator translation speed factor
  private double elevHeightStrafeFactor; // Keeps track of the elevator strafe speed factor
  private double elevHeightRotationFactor; // Keeps track of the elevator rotation speed factor

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      CoralSubsystem coral,
      ElevatorSubsystem elevator,
      AlgaeSubsystem algae,
      VisionSubsystem vision,
      RobotContainer container) {
    this.swerve = swerve;
    this.coral = coral;
    this.elevator = elevator;
    this.algae = algae;
    this.vision = vision;
    this.container = container;

    // Instatiate swerve max speed and angular rate //
    maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
    maxAngularRate =
        RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second max angular velocity

    // Instantiate vision PID controllers //
    visionRangePID =
        new PIDController(
            VisionConstants.RANGE_P, VisionConstants.RANGE_I, VisionConstants.RANGE_D);
    visionRangePID.setTolerance(VisionConstants.RANGE_TOLERANCE);
    visionAimPID =
        new PIDController(VisionConstants.AIM_P, VisionConstants.AIM_I, VisionConstants.AIM_D);
    visionAimPID.setTolerance(VisionConstants.AIM_TOLERANCE);

    // Instantiate default field centric drive (no need to maintain heading) //
    drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05) // Add a 5% deadband
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping

    // Instantiate brake (X-lock swerve wheels) //
    brake = new SwerveRequest.SwerveDriveBrake();

    // Instantiate point (point swerve wheels in a specific direction) //
    point = new SwerveRequest.PointWheelsAt();

    // Instantiate robot centric drive (forward is based on forward pose of the
    // robot) //
    driveRobotCentric =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Instantiate field centric drive (maintain heading) //
    driveMaintainHeading =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.05)
            .withRotationalDeadband(maxAngularRate * 0.05)
            .withDriveRequestType(
                DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withDesaturateWheelSpeeds(true);

    // Set the PID constants for the Maintain Heading controller //
    driveMaintainHeading.HeadingController.setPID(
        SwerveConstants.HEADING_KP, SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD);
    driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveMaintainHeading.HeadingController.setTolerance(SwerveConstants.HEADING_TOLERANCE);

    // Instantiate the Field and Robot Speeds Swerve Requests //
    applyFieldSpeeds =
        new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);
    applyRobotSpeeds =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(DriveRequestType.Velocity);

    // Instantiate current and wanted super states as stopped //
    reefAlignmentState = ReefAlignmentStates.ALIGN_REEF_LEFT_L1;

    // Instantiate current heading as empty //
    currentHeading = Optional.empty(); // Keeps track of current heading

    // Instantiate the rotation last triggered as 0 //
    rotationLastTriggered = 0.0;

    // Instantiate the elevator wanted state as home //
    elevState = ElevatorSubsystem.ElevatorState.HOME;

    // Initialize the elevator height speed factor //
    elevHeightTranslationFactor = 1.0;
    elevHeightStrafeFactor = 1.0;
    elevHeightRotationFactor = 1.0;

    // Instantiate the logger for telemetry //
    logger = new Telemetry(maxSpeed);

    // Register the telemetry for the swerve drive //
    swerve.registerTelemetry(logger::telemeterize);
  }

  // Swerve Drive Subsystem Commands //

  public Command DefaultDriveCommand(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup) {
    return new RunCommand(
        () -> {
          double rawTranslation = translationSup.getAsDouble();
          double rawStrafe = strafeSup.getAsDouble();
          double rawRotation = rotationSup.getAsDouble();

          // Apply deadband to joystick inputs //
          double translation =
              MathUtil.applyDeadband(
                  rawTranslation, SwerveConstants.SWERVE_DEADBAND, 1); // Forward/Backward
          double strafe =
              MathUtil.applyDeadband(rawStrafe, SwerveConstants.SWERVE_DEADBAND, 1); // Left/Right
          double rotation =
              MathUtil.applyDeadband(rawRotation, SwerveConstants.SWERVE_DEADBAND, 1); // Rotation

          // Square the inputs (while preserving the sign) to increase fine control while
          // permitting full power //
          translation = Math.copySign(translation * translation, translation);
          strafe = Math.copySign(strafe * strafe, strafe);
          rotation = Math.copySign(rotation * rotation, rotation);

          // Multiply the translation, strafe, and rotation by the elevator height speed factor //
          translation *= elevHeightTranslationFactor;
          strafe *= elevHeightStrafeFactor;
          rotation *= elevHeightRotationFactor;

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
          boolean rotationActive =
              MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1)
                  && (Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond)
                      > Math.toRadians(10));

          // Check if the rotation of the joystick has been triggered //
          if (rotationTriggered) {
            // Timestamp the last time the rotation was triggered //
            rotationLastTriggered = Timer.getFPGATimestamp();
          }

          if (rotationTriggered || rotationActive) {
            // If the rotation is triggered or active, set the swerve drive to rotate in
            // place //
            swerve.setControl(
                drive
                    .withVelocityX(translation)
                    .withVelocityY(strafe)
                    .withRotationalRate(rotation));
            currentHeading = Optional.empty();
          } else {
            // Initialize the current heading if it is empty
            if (currentHeading.isEmpty()) {
              currentHeading = Optional.of(swerve.getState().Pose.getRotation());
            }

            Optional<Alliance> alliance = DriverStation.getAlliance();

            if (alliance.isPresent()) {
              swerve.setControl(
                  driveMaintainHeading
                      .withVelocityX(translation)
                      .withVelocityY(strafe)
                      .withTargetDirection(
                          alliance.get() == Alliance.Blue
                              ? currentHeading.get()
                              : currentHeading.get().rotateBy(Rotation2d.fromDegrees(180))));
            }
          }
        },
        swerve);
  }

  public Command DriveToClosestReefPoseCommand(boolean left) {

    return new DeferredCommand(
            () -> {
              // Grab the robot's current alliance
              Optional<Alliance> alliance = DriverStation.getAlliance();

              // Grab the robot's current pose
              Pose2d currentPose = swerve.getState().Pose;

              // Initialize the target poses based on the alliance and whether we are left or right
              //
              final var redReefPoses = left
                  ? FieldConstants.Reef.RED_REEF_STATION_LEFT_POSES
                  : FieldConstants.Reef.RED_REEF_STATION_RIGHT_POSES;
              final var blueReefPoses = left
                  ? FieldConstants.Reef.BLUE_REEF_STATION_LEFT_POSES
                  : FieldConstants.Reef.BLUE_REEF_STATION_RIGHT_POSES;
              List<Pose2d> targetPoses =
                  alliance.isPresent() && alliance.get() == Alliance.Red
                      ? redReefPoses
                      : blueReefPoses;

              // Calculate the pose closest to the current pose
              Pose2d closestPose = findClosestPose(currentPose, targetPoses);

              // Add intermediate waypoint (0.25 meter back from target)
              Transform2d backwardOffset = new Transform2d(-0.25, 0.0, Rotation2d.kZero);

              return new DriveToPosePID(
                  swerve,
                  applyRobotSpeeds,
                  closestPose.transformBy(backwardOffset))
              .andThen(
                  new ParallelDeadlineGroup(
                      new DriveToPosePID(
                          swerve,
                          applyRobotSpeeds,
                          closestPose),
                      this.ElevatorL3()));
            },
            Set.of(swerve))
        .andThen(() -> currentHeading = Optional.of(swerve.getState().Pose.getRotation()));
  }

  public Command DriveToClosestCoralStationPoseCommand() {

    return new DeferredCommand(
            () -> {
              // Grab the robot's current alliance
              Optional<Alliance> alliance = DriverStation.getAlliance();

              // Grab the robot's current pose
              Pose2d currentPose = swerve.getState().Pose;

              List<Pose2d> targetPoses =
                  alliance.isPresent() && alliance.get() == Alliance.Red
                      ? FieldConstants.CoralStation.RED_CORAL_STATION_POSES
                      : FieldConstants.CoralStation.BLUE_CORAL_STATION_POSES;

              Pose2d closestPose = findClosestPose(currentPose, targetPoses);

              return new DriveToPosePID(
                  swerve,
                  applyRobotSpeeds,
                  closestPose);
            },
            Set.of(swerve))
        .andThen(() -> currentHeading = Optional.of(swerve.getState().Pose.getRotation()));
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

  public void initCurrentHeading() {
    currentHeading = Optional.of(swerve.getState().Pose.getRotation());
  }

  public Command SwerveBrake() {
    return swerve.applyRequest(() -> brake);
  }

  public Command SeedFieldCentric() {
    return swerve.runOnce(swerve::seedFieldCentric);
  }

  // Elevator Subsystem Commands //

  public Command ElevatorZero() {
    return new InstantCommand(
        () -> elevator.seedElevatorMotorEncoderPosition(0),
        elevator);
  }

  public Command ElevatorStop() {
    return new InstantCommand(
        () -> elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE),
        elevator);
  }

  public Command ElevatorManualDown() {
    return new RunCommand(
        () -> elevator.setElevatorMotorSpeed(-0.5),
        elevator);
  }

  public Command ElevatorManualUp() {
    return new RunCommand(
        () -> elevator.setElevatorMotorSpeed(0.5),
        elevator);
  }

  public Command ElevatorIdle() {
    Command resetEncoder =
        new InstantCommand(
            () -> elevator.seedElevatorMotorEncoderPosition(0),
            elevator);

    Command setElevatorToIdle =
        new InstantCommand(
            () -> elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE),
            elevator);

    return new ConditionalCommand(
        resetEncoder.andThen(setElevatorToIdle),
        setElevatorToIdle,
        elevator::isAtElevatorState);
  }

  public Command ElevatorHome() {

    Command setElevatorHome =
        new InstantCommand(
            () -> {
              // Set the robot back to full speed //
              elevHeightTranslationFactor = 1.0;
              elevHeightStrafeFactor = 1.0;
              elevHeightRotationFactor = 1.0;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.HOME);
            },
            elevator);

    Command waitUntilElevatorIsAtBottom =
        new WaitUntilCommand(elevator::isAtElevatorState);

    Command reZeroElevatorEncoder =
        new InstantCommand(
            () -> elevator.seedElevatorMotorEncoderPosition(0),
            elevator);

    return setElevatorHome.andThen(waitUntilElevatorIsAtBottom).andThen(reZeroElevatorEncoder);

    // If the elevator is at the home position, check if there is a current spike //
    // If there is a current spike, reset the elevator encoder, then set to idle //
    // else set the motor to idle
  }

  public Command ElevatorL1() {

    Command setElevatorL1 =
        new InstantCommand(
            () -> {
              elevHeightTranslationFactor = 1.0;
              elevHeightStrafeFactor = 1.0;
              elevHeightRotationFactor = 1.0;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
            },
            elevator);

    Command waitUntilElevatorIsAtL1 = new WaitUntilCommand(elevator::isAtElevatorState);

    return setElevatorL1.andThen(waitUntilElevatorIsAtL1);
  }

  public Command ElevatorL2() {

    Command setElevatorL2 =
        new InstantCommand(
            () -> {
              elevHeightTranslationFactor = 0.75;
              elevHeightStrafeFactor = 0.75;
              elevHeightRotationFactor = 1;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L2);
            },
            elevator);

    Command waitUntilElevatorIsAtL2 = new WaitUntilCommand(elevator::isAtElevatorState);

    return setElevatorL2.andThen(waitUntilElevatorIsAtL2);
  }

  public Command ElevatorL3() {

    Command setElevatorL3 =
        new InstantCommand(
            () -> {
              elevHeightTranslationFactor = 0.65;
              elevHeightStrafeFactor = 0.65;
              elevHeightRotationFactor = 1;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L3);
            },
            elevator);

    Command waitUntilElevatorIsAtL3 = new WaitUntilCommand(elevator::isAtElevatorState);

    return setElevatorL3.andThen(waitUntilElevatorIsAtL3);
  }

  public Command ElevatorL4() {

    Command setElevatorL4 =
        new InstantCommand(
            () -> {
              elevHeightTranslationFactor = 0.5;
              elevHeightStrafeFactor = 0.5;
              elevHeightRotationFactor = 1;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L4);
            },
            elevator);

    Command waitUntilElevatorIsAtL4 = new WaitUntilCommand(elevator::isAtElevatorState);

    return setElevatorL4.andThen(waitUntilElevatorIsAtL4);
  }

  // Coral Subsystem Commands //

  public Command IntakeCoral() {

    Command engageCoralIntake =
        new InstantCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.INTAKE), coral);

    Command runUntilCoralIsDetected = new WaitUntilCommand(() -> coral.isBeamBreakTripped());

    Command slowIntake =
        new InstantCommand(
            () -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_INTAKE), coral);

    Command runUntilCoralIsNotDetected = new WaitUntilCommand(() -> !coral.isBeamBreakTripped());

    Command slowReverseIntake =
        new InstantCommand(
            () -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE), coral);

    Command runUntilCoralIsDetectedAgain = new WaitUntilCommand(() -> coral.isBeamBreakTripped());

    Command slowIntakeAgain =
        new InstantCommand(
            () -> coral.setCoralState(CoralSubsystem.CoralState.SLOWER_INTAKE), coral);

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
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE), coral);
  }

  public Command ScoreCoral() {
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SCORE), coral);
  }

  public Command HoldCoral() {
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.HOLD), coral);
  }

  // Algae Subsystem Commands //

  public Command IntakeAlgae() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), algae);
  }

  public Command DeAlgaeify() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), algae);
  }

  public Command ScoreAlgae() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.SCORE), algae);
  }

  public Command AlgaeArmHome() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOME), algae);
  }

  public Command AlgaeArmIntake() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), algae);
  }

  public Command AlgaeArmDeAlgaeify() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), algae);
  }

  public Command AlgaeArmHold() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD), algae);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the robot pose using Photonvision's Pose Estimates //
  }
}
