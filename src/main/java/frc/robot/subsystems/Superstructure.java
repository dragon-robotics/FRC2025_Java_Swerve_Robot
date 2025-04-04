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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

  /* Used for path following */
  private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds;
  private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds;

  /* Holonomic Drive Controller for pathfollowing */
  private HolonomicDriveController holonomicDriveController;
  private PIDController xController;
  private PIDController yController;
  private ProfiledPIDController thetaController;

  private final double maxSpeed;
  private final double maxAngularRate;
  private PIDController visionRangePID;
  private PIDController visionAimPID;

  private double rotationLastTriggered; // Keeps track of the last time the rotation was triggered
  private Optional<Rotation2d> currentHeading; // Keeps track of current heading

  private boolean useLeftCamera; // Keeps track of the camera to use
  private boolean intakeCoralLeft; // Keeps track of the coral intake side
  private ElevatorSubsystem.ElevatorState
      elevatorState; // Keeps track of the current elevator state
  private double
      elevatorHeightTranslationFactor; // Keeps track of the elevator translation speed factor
  private double elevatorHeightStrafeFactor; // Keeps track of the elevator strafe speed factor
  private double elevatorHeightRotationFactor; // Keeps track of the elevator rotation speed factor

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

    // Instantiate the holonomic drive controller for path following //
    xController = new PIDController(10, 0, 0);
    yController = new PIDController(10, 0, 0);
    thetaController =
        new ProfiledPIDController(7, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI / 2));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    holonomicDriveController =
        new HolonomicDriveController(xController, yController, thetaController);

    // Instantiate current and wanted super states as stopped //
    m_reefAlignmentState = ReefAlignmentStates.ALIGN_REEF_LEFT_L1;

    // Instantiate current heading as empty //
    currentHeading = Optional.empty(); // Keeps track of current heading

    // Instantiate the rotation last triggered as 0 //
    rotationLastTriggered = 0.0;

    // Instantiate the elevator wanted state as home //
    elevatorState = ElevatorSubsystem.ElevatorState.HOME;

    // Default we use the right camera //
    useLeftCamera = true;

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
    return new RunCommand(
        () -> {
          double rawTranslation = translationSup.getAsDouble();
          double rawStrafe = strafeSup.getAsDouble();
          double rawRotation = rotationSup.getAsDouble();

          // Apply deadband to joystick inputs //
          MathUtil.applyDeadband(rawTranslation, SwerveConstants.SWERVE_DEADBAND, 0.1);
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
          boolean rotationActive =
              MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1)
                  && (Math.abs(m_swerve.getState().Speeds.omegaRadiansPerSecond)
                      > Math.toRadians(10));

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
                          alliance.get() == Alliance.Blue
                              ? currentHeading.get()
                              : currentHeading.get().rotateBy(Rotation2d.fromDegrees(180))));
            }
          }
        },
        m_swerve);
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

  public Command ElevatorIdle() {
    Command resetEncoder =
        new InstantCommand(
            () -> {
              m_elevator.seedElevatorMotorEncoderPosition(0);
            });

    Command setElevatorToIdle =
        new InstantCommand(
            () -> {
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE);
            },
            m_elevator);

    Command resetElevatorEncoder =
        new ConditionalCommand(
            resetEncoder.andThen(setElevatorToIdle),
            setElevatorToIdle,
            m_elevator::isAtElevatorState);

    return resetElevatorEncoder;
  }

  public Command ElevatorHome() {

    Command setElevatorHome =
        new InstantCommand(
            () -> {
              // Set the robot back to full speed //
              elevatorHeightTranslationFactor = 1.0;
              elevatorHeightStrafeFactor = 1.0;
              elevatorHeightRotationFactor = 1.0;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.HOME);
            },
            m_elevator);

    Command waitUntilElevatorIsAtBottom =
        new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    Command reZeroElevatorEncoder =
        new InstantCommand(
            () -> {
              m_elevator.seedElevatorMotorEncoderPosition(0);
            },
            m_elevator);

    return setElevatorHome.andThen(waitUntilElevatorIsAtBottom).andThen(reZeroElevatorEncoder);

    // If the elevator is at the home position, check if there is a current spike //
    // If there is a current spike, reset the elevator encoder, then set to idle //
    // else set the motor to idle
  }

  public Command ElevatorL1() {

    Command setElevatorL1 =
        new InstantCommand(
            () -> {
              elevatorHeightTranslationFactor = 1.0;
              elevatorHeightStrafeFactor = 1.0;
              elevatorHeightRotationFactor = 1.0;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
            },
            m_elevator);

    Command waitUntilElevatorIsAtL1 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL1.andThen(waitUntilElevatorIsAtL1);
  }

  public Command ElevatorL2() {

    Command setElevatorL2 =
        new InstantCommand(
            () -> {
              elevatorHeightTranslationFactor = 0.75;
              elevatorHeightStrafeFactor = 0.75;
              elevatorHeightRotationFactor = 1;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L2);
            },
            m_elevator);

    Command waitUntilElevatorIsAtL2 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL2.andThen(waitUntilElevatorIsAtL2);
  }

  public Command ElevatorL3() {

    Command setElevatorL3 =
        new InstantCommand(
            () -> {
              elevatorHeightTranslationFactor = 0.65;
              elevatorHeightStrafeFactor = 0.65;
              elevatorHeightRotationFactor = 1;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L3);
            },
            m_elevator);

    Command waitUntilElevatorIsAtL3 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL3.andThen(waitUntilElevatorIsAtL3);
  }

  public Command ElevatorL4() {

    Command setElevatorL4 =
        new InstantCommand(
            () -> {
              elevatorHeightTranslationFactor = 0.5;
              elevatorHeightStrafeFactor = 0.5;
              elevatorHeightRotationFactor = 1;
              m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L4);
            },
            m_elevator);

    Command waitUntilElevatorIsAtL4 = new WaitUntilCommand(() -> m_elevator.isAtElevatorState());

    return setElevatorL4.andThen(waitUntilElevatorIsAtL4);
  }

  // Coral Subsystem Commands //
  public Command SetCoralStation(boolean left) {
    Command setCoralStation =
        new InstantCommand(
            () -> {
              intakeCoralLeft = left;
            });

    return setCoralStation;
  }

  public Command IntakeCoral() {

    // Command setRobotHeading = new InstantCommand(() -> {
    //   // Set the robot heading to the coral station angle for intake //
    //   currentHeading = intakeCoralLeft ?
    //     Optional.of(GeneralConstants.LEFT_CORAL_STATION_INTAKE_ANGLE) : // If left, set to left
    // side angle
    //     Optional.of(GeneralConstants.RIGHT_CORAL_STATION_INTAKE_ANGLE); // If right, set to right
    // side angle
    // });

    Command engageCoralIntake =
        new InstantCommand(() -> m_coral.setCoralState(CoralSubsystem.CoralState.INTAKE), m_coral);

    Command runUntilCoralIsDetected = new WaitUntilCommand(() -> m_coral.isBeamBreakTripped());

    Command slowIntake =
        new InstantCommand(
            () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOW_INTAKE), m_coral);

    Command runUntilCoralIsNotDetected = new WaitUntilCommand(() -> !m_coral.isBeamBreakTripped());

    Command slowReverseIntake =
        new InstantCommand(
            () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOW_REVERSE), m_coral);

    Command runUntilCoralIsDetectedAgain = new WaitUntilCommand(() -> m_coral.isBeamBreakTripped());

    Command slowIntakeAgain =
        new InstantCommand(
            () -> m_coral.setCoralState(CoralSubsystem.CoralState.SLOWER_INTAKE), m_coral);

    // Run until the beambreak or current limit is tripped //
    return engageCoralIntake
        .andThen(runUntilCoralIsDetected)
        .andThen(slowIntake)
        .andThen(runUntilCoralIsNotDetected)
        .andThen(slowReverseIntake)
        // .andThen(new WaitCommand(0.1));
        .andThen(runUntilCoralIsDetectedAgain)
        .andThen(slowIntakeAgain)
        .andThen(new WaitCommand(0.02));
  }

  // public Command AimAndRangeReefApriltag() {

  //   Command autoAlignToReefTag =
  //       new AutoAlignToReefTag(
  //           useLeftCamera, visionAimPID, visionRangePID, driveMaintainHeading, m_vision,
  // m_swerve);

  //   Command setCurrentHeading =
  //       new InstantCommand(
  //           () -> {
  //             currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
  //           });

  //   return autoAlignToReefTag.andThen(setCurrentHeading);
  // }

  // public Command AlignToScoreCoral() {

  //   Command autoDriveToReefStation = AimAndRangeReefApriltag();

  //   Command setElevatorLevel =
  //       new InstantCommand(
  //           () -> {
  //             m_elevator.setElevatorState(ElevatorSubsystem.ElevatorState.L1);
  //           },
  //           m_elevator);

  //   Command waitUntilElevatorIsAtLevel = new WaitUntilCommand(() ->
  // m_elevator.isAtElevatorState());

  //   return autoDriveToReefStation.andThen(setElevatorLevel).andThen(waitUntilElevatorIsAtLevel);
  // }

  public Command ScoreCoral() {
    Command scoreCoral =
        new RunCommand(() -> m_coral.setCoralState(CoralSubsystem.CoralState.SCORE), m_coral);

    return scoreCoral;
  }

  public Command HoldCoral() {
    Command holdCoral =
        new RunCommand(() -> m_coral.setCoralState(CoralSubsystem.CoralState.HOLD), m_coral);

    return holdCoral;
  }

  // Algae Subsystem Commands //

  public Command IntakeAlgae() {

    Command engageAlgaeIntake =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), m_algae);

    Command runUntilAlgaeIsDetected = new WaitUntilCommand(() -> m_algae.hasAlgae());

    Command holdAlgae =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD), m_algae);

    // Run until the beambreak or current limit is tripped //
    return engageAlgaeIntake;
  }

  public Command DeAlgaeify() {
    Command engageAlgaeDeAlgaeify =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), m_algae);

    return engageAlgaeDeAlgaeify;
  }

  public Command ScoreAlgae() {
    Command scoreAlgae =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.SCORE), m_algae);

    return scoreAlgae;
  }

  public Command AlgaeArmHome() {
    Command homeAlgaeArm =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOME), m_algae);

    return homeAlgaeArm;
  }

  public Command AlgaeArmIntake() {
    Command setAlgaeArmIntake =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.INTAKE), m_algae);

    return setAlgaeArmIntake;
  }

  public Command AlgaeArmDeAlgaeify() {
    Command setAlgaeArmDeAlgaeify =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.DEALGAE), m_algae);

    return setAlgaeArmDeAlgaeify;
  }

  public Command AlgaeArmHold() {
    Command setAlgaeArmHold =
        new RunCommand(() -> m_algae.setAlgaeState(AlgaeSubsystem.AlgaeState.HOLD), m_algae);

    return setAlgaeArmHold;
  }

  /** Robot state configurations - operator (2nd driver) sets up what the score button does */
  public void setReefAlignment(ReefAlignmentStates wantedReefAlignmentState) {
    switch (wantedReefAlignmentState) {
      case ALIGN_REEF_LEFT_L1:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = true; // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L2:
        elevatorState = ElevatorSubsystem.ElevatorState.L2; // Set the elevator to L2
        useLeftCamera = true; // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L3:
        elevatorState = ElevatorSubsystem.ElevatorState.L3; // Set the elevator to L1
        useLeftCamera = true; // Use the left camera
        break;
      case ALIGN_REEF_LEFT_L4:
        elevatorState = ElevatorSubsystem.ElevatorState.L4; // Set the elevator to L1
        useLeftCamera = true; // Use the left camera
        break;
      case ALIGN_REEF_RIGHT_L1:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = false; // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L2:
        elevatorState = ElevatorSubsystem.ElevatorState.L2; // Set the elevator to L1
        useLeftCamera = false; // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L3:
        elevatorState = ElevatorSubsystem.ElevatorState.L3; // Set the elevator to L3
        useLeftCamera = false; // Use the right camera
        break;
      case ALIGN_REEF_RIGHT_L4:
        elevatorState = ElevatorSubsystem.ElevatorState.L4; // Set the elevator to L4
        useLeftCamera = false; // Use the right camera
        break;
      default:
        elevatorState = ElevatorSubsystem.ElevatorState.L1; // Set the elevator to L1
        useLeftCamera = true; // Use the left camera
        break;
    }
  }

  public Command setReefAlignmentCommand(ReefAlignmentStates wantedReefAlignmentState) {
    return new InstantCommand(() -> setReefAlignment(wantedReefAlignmentState));
  }

  public void setReefAlignment(boolean left) {
    useLeftCamera = left;
  }

  public Command SetReefAlignment(boolean left) {
    return new InstantCommand(() -> setReefAlignment(left));
  }

  public void setReefHeight(ElevatorState wantedElevatorState) {
    elevatorState = wantedElevatorState;
  }

  public Command SetReefHeight(ElevatorState wantedElevatorState) {
    return new InstantCommand(() -> setReefHeight(wantedElevatorState));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update the robot pose using Photonvision's Pose Estimates //
  }
}
