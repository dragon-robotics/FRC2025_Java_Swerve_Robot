// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.RobotContainer;
import frc.robot.commands.teleop.DriveToPosePID;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.controller.ControllerSubsystem;
import frc.robot.subsystems.controller.ControllerSubsystem.ControllerState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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
  private final ControllerSubsystem controller;
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
  private double
      elevHeightTranslationFactor; // Keeps track of the elevator translation speed factor
  private double elevHeightStrafeFactor; // Keeps track of the elevator strafe speed factor
  private double elevHeightRotationFactor; // Keeps track of the elevator rotation speed factor

  // Cache the closest reef pose, updated periodically
  private Pose2d cachedClosestLeftReef = new Pose2d();
  private Pose2d cachedClosestRightReef = new Pose2d();
  private Pose2d cachedClosestCoralStation = new Pose2d();

  private int updateCounter = 0;
  private static final int CACHED_CLOSEST_POSE_UPDATE_RATE = 5; // Update every 5th call to periodic()

  /** Creates a new Superstructure. */
  public Superstructure(
      CommandSwerveDrivetrain swerve,
      CoralSubsystem coral,
      ElevatorSubsystem elevator,
      AlgaeSubsystem algae,
      VisionSubsystem vision,
      ControllerSubsystem controller,
      RobotContainer container) {
    this.swerve = swerve;
    this.coral = coral;
    this.elevator = elevator;
    this.algae = algae;
    this.vision = vision;
    this.controller = controller;
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

  public Command defaultDriveCmd(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier halfSpeedSup) {
    return new RunCommand(
        () -> {
          double rawRotation = rotationSup.getAsDouble();
          var speeds =
              processJoystickInputs(
                  translationSup.getAsDouble(),
                  strafeSup.getAsDouble(),
                  rawRotation,
                  halfSpeedSup.getAsBoolean());

          boolean rotationTriggered = Math.abs(rawRotation) > SwerveConstants.SWERVE_DEADBAND;
          boolean rotationActive =
              MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1)
                  && (Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond)
                      > Math.toRadians(10));

          if (rotationTriggered) {
            rotationLastTriggered = Timer.getFPGATimestamp();
          }

          if (rotationTriggered || rotationActive) {
            setSwerveToRotate(speeds.translation, speeds.strafe, speeds.rotation);
          } else {
            setSwerveToMaintainHeading(speeds.translation, speeds.strafe);
          }
        },
        swerve);
  }

  private static class Speeds {
    double translation;
    double strafe;
    double rotation;
  }

  private Speeds processJoystickInputs(
      double rawTranslation, double rawStrafe, double rawRotation, boolean halfSpeed) {
    double translation = MathUtil.applyDeadband(rawTranslation, SwerveConstants.SWERVE_DEADBAND, 1);
    double strafe = MathUtil.applyDeadband(rawStrafe, SwerveConstants.SWERVE_DEADBAND, 1);
    double rotation = MathUtil.applyDeadband(rawRotation, SwerveConstants.SWERVE_DEADBAND, 1);

    translation = Math.copySign(translation * translation, translation);
    strafe = Math.copySign(strafe * strafe, strafe);
    rotation = Math.copySign(rotation * rotation, rotation);

    translation *= elevHeightTranslationFactor;
    strafe *= elevHeightStrafeFactor;
    rotation *= elevHeightRotationFactor;

    if (halfSpeed) {
      translation *= 0.35;
      strafe *= 0.35;
      rotation *= 0.35;
    }

    Speeds speeds = new Speeds();
    speeds.translation = translation * maxSpeed;
    speeds.strafe = strafe * maxSpeed;
    speeds.rotation = rotation * maxAngularRate;
    return speeds;
  }

  private void setSwerveToRotate(double translation, double strafe, double rotation) {
    swerve.setControl(
        drive.withVelocityX(translation).withVelocityY(strafe).withRotationalRate(rotation));
    currentHeading = Optional.empty();
  }

  private void setSwerveToMaintainHeading(double translation, double strafe) {
    if (currentHeading.isEmpty()) {
      currentHeading = Optional.of(swerve.getState().Pose.getRotation());
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      Rotation2d targetDirection =
          alliance.get() == Alliance.Blue
              ? currentHeading.get()
              : currentHeading.get().rotateBy(Rotation2d.fromDegrees(180));
      swerve.setControl(
          driveMaintainHeading
              .withVelocityX(translation)
              .withVelocityY(strafe)
              .withTargetDirection(targetDirection));
    }
  }

  public Command driveToClosestReefPoseCmd(boolean left) {
    return new DeferredCommand(
            () -> {
              Pose2d target = left ? cachedClosestLeftReef : cachedClosestRightReef;
              Transform2d offset = new Transform2d(-0.25, 0.0, Rotation2d.kZero);

              return new DriveToPosePID(swerve, applyRobotSpeeds, target.transformBy(offset))
                  .andThen(new DriveToPosePID(swerve, applyRobotSpeeds, target));
            },
            Set.of(swerve))
        .andThen(
            Commands.deadline(
                new RunCommand(
                        () -> controller.setControllerState(ControllerState.STRONG_RUMBLE),
                        controller)
                    .withTimeout(0.5)),
            new InstantCommand(
                () -> currentHeading = Optional.of(swerve.getState().Pose.getRotation())))
        .handleInterrupt(() -> currentHeading = Optional.of(swerve.getState().Pose.getRotation()));
  }

  public Command driveToClosestCoralStationPoseCmd() {

    return new DeferredCommand(
            () -> new DriveToPosePID(swerve, applyRobotSpeeds, cachedClosestCoralStation),
            Set.of(swerve))
        .andThen(
            Commands.deadline(
                new RunCommand(
                        () -> controller.setControllerState(ControllerState.STRONG_RUMBLE),
                        controller)
                    .withTimeout(0.5)),
            new InstantCommand(
                () -> currentHeading = Optional.of(swerve.getState().Pose.getRotation())))
        .handleInterrupt(() -> currentHeading = Optional.of(swerve.getState().Pose.getRotation()));
  }

  public Command driveToClosestReefPoseWithElevatorCmd(boolean left) {
    return new DeferredCommand(
            () -> {
              Pose2d target = left ? cachedClosestLeftReef : cachedClosestRightReef;

              double offsetDist = -0.25;
              Command elevCmd = this.testElevatorHomeCmd();
              if (elevState == ElevatorSubsystem.ElevatorState.L2) {
                offsetDist = -0.3;
                elevCmd = this.elevatorL2Cmd();
              } else if (elevState == ElevatorSubsystem.ElevatorState.L3) {
                offsetDist = -0.4;
                elevCmd = this.elevatorL3Cmd();
              } else if (elevState == ElevatorSubsystem.ElevatorState.L4) {
                offsetDist = -0.5;
                elevCmd = this.elevatorL4Cmd();
              }

              Transform2d offset = new Transform2d(offsetDist, 0.0, Rotation2d.kZero);

              return new DriveToPosePID(swerve, applyRobotSpeeds, target.transformBy(offset))
                  .andThen(
                      Commands.parallel(
                          elevCmd.withTimeout(0.2),
                          new DriveToPosePID(swerve, applyRobotSpeeds, target)
                      )
                  );
            },
            Set.of(swerve))
        .andThen(
            Commands.deadline(
                new RunCommand(
                        () -> controller.setControllerState(ControllerState.STRONG_RUMBLE),
                        controller)
                    .withTimeout(0.5)),
            new InstantCommand(
                () -> currentHeading = Optional.of(swerve.getState().Pose.getRotation())))
        .handleInterrupt(() -> currentHeading = Optional.of(swerve.getState().Pose.getRotation()));
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

  public Command swerveBrakeCmd() {
    return swerve.applyRequest(() -> brake);
  }

  public Command seedFieldCentricCmd() {
    return swerve.runOnce(swerve::seedFieldCentric);
  }

  // Elevator Subsystem Commands //

  public Command elevatorZeroCmd() {
    return new InstantCommand(() -> elevator.seedElevatorMotorEncoderPosition(0), elevator);
  }

  public Command elevatorStopCmd() {
    return new InstantCommand(
        () -> elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE), elevator);
  }

  public Command elevatorManualDownCmd() {
    return new RunCommand(() -> elevator.setElevatorMotorSpeed(-0.5), elevator);
  }

  public Command elevatorManualUpCmd() {
    return new RunCommand(() -> elevator.setElevatorMotorSpeed(0.5), elevator);
  }

  public Command elevatorIdleCmd() {
    Command resetEncoder =
        new InstantCommand(() -> elevator.seedElevatorMotorEncoderPosition(0), elevator);

    Command setElevatorToIdle =
        new InstantCommand(
            () -> elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE), elevator);

    return new ConditionalCommand(
        resetEncoder.andThen(setElevatorToIdle), setElevatorToIdle, elevator::isAtElevatorState);
  }

  public Command setElevatorStateCmd(ElevatorSubsystem.ElevatorState state) {
    return new InstantCommand(() -> {
      elevState = state;
      DogLog.log("Elevator/ElevatorState", elevState);
    });
  }

  public Command testElevatorHomeCmd() {

    Command setElevatorHome =
        new InstantCommand(
            () -> {
              elevHeightTranslationFactor = 1.0;
              elevHeightStrafeFactor = 1.0;
              elevHeightRotationFactor = 1.0;
              elevator.setElevatorState(ElevatorSubsystem.ElevatorState.HOME);
            },
            elevator);

    Command waitUntilElevatorIsAtHome = new WaitUntilCommand(elevator::isAtElevatorState);

    Command setElevatorToIdle =
        new InstantCommand(
            () -> elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE), elevator);

    Command reZeroElevatorEncoder =
        new InstantCommand(() -> elevator.seedElevatorMotorEncoderPosition(0), elevator);

    // When the elevator goes home, it checks for a current spike to to see if the elevator is at the limit. If not, set the elevator to idle //
    Command elevatorHomeState =
        new ConditionalCommand(
            reZeroElevatorEncoder.andThen(setElevatorToIdle),
            setElevatorToIdle,
            elevator::isCurrentLimitTripped);

    return setElevatorHome
        .andThen(waitUntilElevatorIsAtHome)
        .andThen(elevatorHomeState);  // If the elevator is at the home position, check if there is a current spike //
  }

  public Command elevatorHomeCmd() {

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

    Command waitUntilElevatorIsAtBottom = new WaitUntilCommand(elevator::isAtElevatorState);

    Command reZeroElevatorEncoder =
        new InstantCommand(() -> elevator.seedElevatorMotorEncoderPosition(0), elevator);

    return setElevatorHome.andThen(waitUntilElevatorIsAtBottom).andThen(reZeroElevatorEncoder);

    // If the elevator is at the home position, check if there is a current spike //
    // If there is a current spike, reset the elevator encoder, then set to idle //
    // else set the motor to idle
  }

  public Command elevatorL1Cmd() {

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

  public Command elevatorL2Cmd() {

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

  public Command elevatorL3Cmd() {

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

  public Command elevatorL4Cmd() {

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

  public Command intakeCoralDetectCmd() {
    Command engageCoralIntake =
        new InstantCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.INTAKE), coral);

    Command runUntilCoralIsDetected = new WaitUntilCommand(coral::isBeamBreakTripped);

    return engageCoralIntake
        .andThen(runUntilCoralIsDetected);
  }

  public Command intakeCoralCompleteCmd() {
    Command slowIntake =
        new InstantCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_INTAKE), coral);

    Command runUntilCoralIsNotDetected = new WaitUntilCommand(() -> !coral.isBeamBreakTripped());

    Command slowReverseIntake =
        new InstantCommand(
            () -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE), coral);

    Command runUntilCoralIsDetectedAgain = new WaitUntilCommand(() -> coral.isBeamBreakTripped());

    Command slowIntakeAgain =
        new InstantCommand(
            () -> coral.setCoralState(CoralSubsystem.CoralState.SLOWER_INTAKE), coral);        

    // Run until the beambreak is not detected at all and reverse //
    return slowIntake
        .andThen(runUntilCoralIsNotDetected)
        .andThen(slowReverseIntake)
        .andThen(runUntilCoralIsDetectedAgain)
        .andThen(slowIntakeAgain)
        .andThen(new WaitCommand(0.02));
  }

  public Command intakeCoralCmd() {

    Command engageCoralIntake =
        new InstantCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.INTAKE), coral);

    Command runUntilCoralIsDetected = new WaitUntilCommand(() -> coral.isBeamBreakTripped());

    Command slowIntake =
        new InstantCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_INTAKE), coral);

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

  public Command reverseCoralIntakeCmd() {
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE), coral);
  }

  public Command scoreCoralCmd() {
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.SCORE), coral);
  }

  public Command holdCoralCmd() {
    return new RunCommand(() -> coral.setCoralState(CoralSubsystem.CoralState.HOLD), coral);
  }

  // Algae Subsystem Commands //

  public Command intakeAlgaeCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), algae);
  }

  public Command deAlgaeifyCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), algae);
  }

  public Command scoreAlgaeCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.SCORE), algae);
  }

  public Command algaeArmHomeCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOME), algae);
  }

  public Command algaeArmIntakeCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), algae);
  }

  public Command algaeArmDeAlgaeifyCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), algae);
  }

  public Command algaeArmHoldCmd() {
    return new RunCommand(() -> algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD), algae);
  }

  public Command defaultControllerRumbleCmd() {
    return new RunCommand(
        () -> controller.setControllerState(ControllerState.NO_RUMBLE), controller);
  }

  private void updateCachedClosestPoses() {
    Pose2d currentPose = swerve.getState().Pose;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      alliance = Optional.of(Alliance.Blue); // Default to Blue if no alliance
    }

    Alliance currentAlliance = alliance.get();
    boolean isRed = currentAlliance == Alliance.Red;

    boolean shouldFlip = currentPose.getX() > FieldConstants.REEF_CENTER_X;

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

    if (shouldFlip) {
      cachedClosestLeftReef = findClosestPose(currentPose, rightReefPoses);
      cachedClosestRightReef = findClosestPose(currentPose, leftReefPoses);
    } else {
      cachedClosestLeftReef = findClosestPose(currentPose, leftReefPoses);
      cachedClosestRightReef = findClosestPose(currentPose, rightReefPoses);
    }
    cachedClosestCoralStation = findClosestPose(currentPose, coralStationPoses);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++updateCounter >= CACHED_CLOSEST_POSE_UPDATE_RATE) {
      updateCounter = 0;
      updateCachedClosestPoses();
    }

    DogLog.log("Elevator/ElevatorState", elevState);
  }
}
