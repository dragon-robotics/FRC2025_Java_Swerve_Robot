// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    STOPPED,  // Default state
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
    STOPPED,  // Default state
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

  private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
  private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    // Instantiate robot centric drive (forward is based on forward pose of the robot) //
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
        SwerveConstants.HEADING_KD
    );
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
    BooleanSupplier halfSpeedSup
  ) {
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
        // Square the inputs (while preserving the sign) to increase fine control while permitting full power //
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
      boolean rotationActive
        = MathUtil.isNear(rotationLastTriggered, Timer.getFPGATimestamp(), 0.1) &&
          (Math.abs(m_swerve.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(10));

      // Check if the rotation of the joystick has been triggered //
      if (rotationTriggered){
        // Timestamp the last time the rotation was triggered //
        rotationLastTriggered = Timer.getFPGATimestamp();
      }

      if (rotationTriggered || rotationActive) {
          // If the rotation is triggered or active, set the swerve drive to rotate in place //
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

  public Command SwerveBrake () {
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

  private void applyState() {
    switch (currentSuperState) {
      case INTAKE_CORAL_LEFT:
        intakeCoral(true);
        break;
      case INTAKE_CORAL_RIGHT:
        intakeCoral(false);
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
  }

  private void intakeCoral(boolean left) {
    if (left) {
      wantedSuperState = WantedSuperState.INTAKE_CORAL_LEFT;
    } else {
      wantedSuperState = WantedSuperState.INTAKE_CORAL_RIGHT;
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
