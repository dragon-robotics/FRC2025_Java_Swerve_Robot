// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Test.TestIntake;
import frc.robot.commands.Test.TestShooter;
import frc.robot.commands.Test.TestUptake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain m_swerveDriveSubsystem = TunerConstants.createDrivetrain(
    250,
    SwerveConstants.ODOMETRY_STD,
    VisionConstants.DEFAULT_TAG_STDDEV);

  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem(m_swerveDriveSubsystem.getState());
  // public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // public final UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  // public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  // public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  // public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  // public final Limelight3Subsystem m_limelight3Subsystem = new Limelight3Subsystem();

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05)
      .withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
      .withDesaturateWheelSpeeds(true); // Desaturate wheel speeds to prevent clipping
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(MaxSpeed * 0.05)
      .withRotationalDeadband(MaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
      .withDesaturateWheelSpeeds(true);

  // Setup Telemetry //
  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Setup Slew Rate Limiters for driving //
  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(MaxSpeed);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(MaxSpeed);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(MaxAngularRate);

  private double joystickLastTouched = 0.0; // When the joystick was last touched
  private Optional<Rotation2d> currentHeading = Optional.empty();

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private final CommandJoystick m_operatorButtonBoxController =
      new CommandJoystick(OperatorConstants.OPERATOR_BUTTON_PORT);

  private final SendableChooser<Command> autoChooser;

  // // Create all the shuffleboard tab for testing //
  // public ShuffleboardTab m_testShuffleboardTab = null;
  // public ShuffleboardTab m_intakeShuffleboardTab = null;
  // public ShuffleboardTab m_uptakeShuffleboardTab = null;
  // public ShuffleboardTab m_shooterShuffleboardTab = null;
  // public ShuffleboardTab m_ampShuffleboardTab = null;
  // public ShuffleboardTab m_climberShuffleboardTab = null;

  // GenericEntry noteIsInIntakeEntry = Shuffleboard.getTab("SmartDashboard")
  //     .add("NoteIsInIntake", false)
  //     .withWidget("Boolean Box")
  //     .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
  //     .getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register Named Commands //
    // Example: NamedCommands.registerCommand("MoveUptake0", new MoveUptake(m_uptakeSubsystem, () -> UPTAKE_STOP));
    // Intake Algae (Until Algae Detected) Command //
    // Score Algae Command //
    // Intake Coral (Until Coral Detected) Command //
    // Score Coral Command //
    // Move to L1 //
    // Move to L2 //
    // Move to L3 //
    // Move to L4 //

    driveHeading.HeadingController.setPID(3, 0.0, 0.5);
    // driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    driveHeading.HeadingController.setTolerance(0.01);

    // Init Auto Chooser //
    autoChooser = AutoBuilder.buildAutoChooser("TestAuto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // #region Xbox Controller Bindings

    m_swerveDriveSubsystem.setDefaultCommand(
        new RunCommand(() -> {

            // Apply deadband to joystick inputs
            double leftY = MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.1) * MaxSpeed; // Forward/Backward
            double leftX = MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.1) * MaxSpeed; // Left/Right
            double rightX = MathUtil.applyDeadband(-m_driverController.getRightX(), 0.1) * MaxAngularRate; // Rotation

            if (Math.abs(rightX) > 0.1) {
                joystickLastTouched = Timer.getFPGATimestamp();
            }
            if (Math.abs(rightX) > 0.1
                    || (MathUtil.isNear(joystickLastTouched, Timer.getFPGATimestamp(), 0.1)
                        && Math.abs(m_swerveDriveSubsystem.getState().Speeds.omegaRadiansPerSecond) > Math.toRadians(10))) {
                m_swerveDriveSubsystem.setControl(drive.withVelocityX(leftY).withVelocityY(leftX).withRotationalRate(rightX));
                currentHeading = Optional.empty();
            } else {
                if (currentHeading.isEmpty()) {
                    currentHeading = Optional.of(m_swerveDriveSubsystem.getState().Pose.getRotation());
                }

                // Based on alliance color, add 180 degrees to the current heading
                Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
                if(alliance == Alliance.Red) {
                  Rotation2d adjHeading = currentHeading.get().plus(Rotation2d.fromDegrees(180));
                  // System.out.println(
                  //     "Current Alliance: " + alliance.toString() +
                  //     " Current Heading: " + currentHeading.get().getDegrees() +
                  //     " Adjusted Heading: " + adjHeading.getDegrees());
                  m_swerveDriveSubsystem.setControl(
                      driveHeading.withVelocityX(leftY)
                      .withVelocityY(leftX)
                      .withTargetDirection(adjHeading));
                } else {
                  m_swerveDriveSubsystem.setControl(driveHeading.withVelocityX(leftY).withVelocityY(leftX)
                          .withTargetDirection(currentHeading.get()));
                }

                m_swerveDriveSubsystem.setControl(driveHeading.withVelocityX(leftY).withVelocityY(leftX)
                .withTargetDirection(currentHeading.get()));


            }  
        }, m_swerveDriveSubsystem)
      );

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST){

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      m_driverController.back().and(m_driverController.y()).whileTrue(m_swerveDriveSubsystem.sysIdDynamic(Direction.kForward));
      m_driverController.back().and(m_driverController.x()).whileTrue(m_swerveDriveSubsystem.sysIdDynamic(Direction.kReverse));
      m_driverController.start().and(m_driverController.y()).whileTrue(m_swerveDriveSubsystem.sysIdQuasistatic(Direction.kForward));
      m_driverController.start().and(m_driverController.x()).whileTrue(m_swerveDriveSubsystem.sysIdQuasistatic(Direction.kReverse));

      // m_intakeSubsystem.setDefaultCommand(new TestIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
      // m_uptakeSubsystem.setDefaultCommand(new TestUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
      // m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.setArmSpeed(0.0), m_armSubsystem));
      // m_shooterSubsystem.setDefaultCommand(new TestShooter(m_shooterSubsystem, () -> -m_operatorController.getRightTriggerAxis(), () -> -m_operatorController.getLeftTriggerAxis()));

      // m_operatorController.a()
      //     .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

      // m_operatorController.b()
      //     .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));

    }

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(m_swerveDriveSubsystem.runOnce(() -> m_swerveDriveSubsystem.seedFieldCentric()));

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().whileTrue(m_swerveDriveSubsystem.applyRequest(() -> brake));
    
    // Bind the X button to snap to 90 degrees
    m_driverController.x().whileTrue(m_swerveDriveSubsystem.applyRequest(() ->
        driveHeading.withTargetDirection(Rotation2d.fromDegrees(90))
            .withVelocityX(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
    ));

    // Bind the Y button to snap to 180 degrees
    m_driverController.y().whileTrue(m_swerveDriveSubsystem.applyRequest(() ->
        driveHeading.withTargetDirection(Rotation2d.fromDegrees(180))
            .withVelocityX(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1) * MaxSpeed) // Drive left with negative X (left)
    ));

    m_driverController.pov(0).whileTrue(m_swerveDriveSubsystem.applyRequest(() ->
      forwardStraight.withVelocityX(0.5).withVelocityY(0))
    );
    m_driverController.pov(180).whileTrue(m_swerveDriveSubsystem.applyRequest(() ->
      forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    );

    // Aim and Range to a target Apriltag //
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> {
      // Read in relevant data from the Camera
      boolean targetVisible = false;
      double targetYaw = 0.0;
      double targetRange = 0.0;
      var results = m_visionSubsystem.getCamera().getAllUnreadResults();
      if (!results.isEmpty()) {
          // Camera processed a new frame since last
          // Get the last one in the list.
          var result = results.get(results.size() - 1);
          if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                  if (target.getFiducialId() == 22) {
                      // Found Tag 17, record its information
                      targetYaw = target.getYaw();
                      target.getSkew();
                      targetRange =
                              PhotonUtils.calculateDistanceToTargetMeters(
                                      Units.inchesToMeters(13.75), // Measured with a tape measure, or in CAD.
                                      0.308, // From 2024 game manual for ID 7
                                      Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                                      Units.degreesToRadians(target.getPitch()));

                      targetVisible = true;
                  }
              }
          }
      }

      // If the target is visible, aim and range to it
      if (targetVisible) {
        // Driver wants auto-alignment to tag 17
        // And, tag 17 is in sight, so we can turn toward it.
        // Override the driver's turn and fwd/rev command with an automatic one
        // That turns toward the tag, and gets the range right.
        double strafe = (0.0 - targetYaw) * 0.01 * 1.0;
        double forward = (0.2 - targetRange) * 0.2 * 1.0;
        // double turn = (0.0 - targetYaw) * 0.01 * MaxAngularRate;

        System.out.println(
            "Strafe: " + Double.toString(strafe) +
            " Forward: " + Double.toString(forward) +
            " TargetYaw: " + Double.toString(targetYaw) +
            " TargetRange: " + Double.toString(targetRange));

        m_swerveDriveSubsystem.setOperatorPerspectiveForward(Rotation2d.fromDegrees(120));
        m_swerveDriveSubsystem.setControl(
            driveHeading
                .withVelocityX(forward)
                .withVelocityY(strafe)
                .withTargetDirection(Rotation2d.kZero));
      }
    }, m_visionSubsystem, m_swerveDriveSubsystem))
    .whileFalse(new InstantCommand(() -> {m_swerveDriveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero);}));

    m_swerveDriveSubsystem.registerTelemetry(logger::telemeterize);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
