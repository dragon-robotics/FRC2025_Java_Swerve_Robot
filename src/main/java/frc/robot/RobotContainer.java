// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CustomButtonBoxConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Teleop.MoveArmToPos;
import frc.robot.commands.Teleop.HoldArmToPosition;
import frc.robot.commands.Teleop.MoveIntake;
import frc.robot.commands.Teleop.MoveIntakeUntilNoteDetected;
import frc.robot.commands.Teleop.MoveIntakeUptake;
import frc.robot.commands.Teleop.MoveIntakeUptakeVoltage;
import frc.robot.commands.Teleop.MoveShooter;
import frc.robot.commands.Teleop.MoveUptake;
import frc.robot.commands.Teleop.MoveIntakeUptakeUntilNoteDetected;
import frc.robot.commands.Test.TestIntake;
import frc.robot.commands.Test.TestShooter;
import frc.robot.commands.Test.TestUptake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.Limelight3Subsystem;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  public final Limelight3Subsystem m_limelight3Subsystem = new Limelight3Subsystem();

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private final CommandJoystick m_operatorButtonBoxController =
      new CommandJoystick(OperatorConstants.OPERATOR_BUTTON_PORT);

  private final SendableChooser<Command> autoChooser;

  // Create all the shuffleboard tab for testing //
  public ShuffleboardTab m_testShuffleboardTab = null;
  public ShuffleboardTab m_intakeShuffleboardTab = null;
  public ShuffleboardTab m_uptakeShuffleboardTab = null;
  public ShuffleboardTab m_shooterShuffleboardTab = null;
  public ShuffleboardTab m_ampShuffleboardTab = null;
  public ShuffleboardTab m_climberShuffleboardTab = null;

  GenericEntry noteIsInIntakeEntry = Shuffleboard.getTab("SmartDashboard")
      .add("NoteIsInIntake", false)
      .withWidget("Boolean Box")
      .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
      .getEntry();

  // This is where the command sequence constants are defined //
  private final double INTAKE_STOP = 0.0;
  private final double UPTAKE_STOP = 0.0;
  private final double SHOOTER_STOP = 0.0;

  private final double INTAKE_REVERSE_FULL_SPEED = 1.0;
  private final double UPTAKE_REVERSE_FULL_SPEED = 1.0;
  private final double SHOOTER_REVERSE_FULL_SPEED = 1.0;

  private final double INTAKE_FULL_SPEED = -1.0;
  private final double UPTAKE_FULL_SPEED = -1.0;
  private final double SHOOTER_FULL_SPEED = -1.0;

  private final double MANUAL_ARM_UP_SPEED = 0.5;
  private final double MANUAL_ARM_DOWN_SPEED = -0.5;

  private final double INTAKE_SLIGHT_REVERSE_SPEED = 0.1;
  private final double INTAKE_UNJAM_SPEED = 0.5;
  private final double INTAKE_UNTIL_NOTE_DETECTED_SPEED = -0.8;
  private final double INTAKE_UNTIL_NOTE_DETECTED_TIMEOUT = 0.25;

  private final double UPTAKE_SHOOT_SPEED = -1.0;
  private final double UPTAKE_SHOOT_TIMEOUT = 1.0;
  private final double UPTAKE_SHOOT_INTAKE_SPEED = -0.5;

  private final double AMP_PREP_ARM_SETPOINT_WAIT_TIME = -5.0;
  private final double AMP_PREP_INTAKE_VOLTAGE = -5.0;
  private final double AMP_PREP_UPTAKE_VOLTAGE = -4.0;
  private final double AMP_PREP_SHOOTER_SPEED = -0.3;
  private final double AMP_PREP_UPTAKE_BUMP_VOLTAGE = -2.5;  // This voltage is to bump the note into the shooter after beambreak activation
  private final double AMP_PREP_BUMP_TIMEOUT = 0.1;  // This timeout is to bump the note into the shooter after beambreak activation

  private final double AMP_SCORE_SHOOTER_SPEED = -0.5;
  private final double AMP_SCORE_SHOOTER_TIMEOUT = 0.5;
  private final double AMP_SCORE_SHOOTER_STOP_TIMEOUT = 0.1;  // This is the timeout to stop the shooter after scoring the note

  private final double FERRY_SHOOTER_SPEED = -0.8;
  private final double FERRY_INTAKE_RELEASE_SPEED = -0.6;
  private final double FERRY_UPTAKE_RELEASE_SPEED = -0.6;
  private final double FERRY_INTAKE_UPTAKE_RELEASE_TIMEOUT = 0.6;
  private final double FERRY_SHOOTER_STOP_TIMEOUT = 0.6;

  private final double PRIME_UPTAKE_SPEAKER_SHOT_INTAKE_REVERSE_SPEED = 0.2;
  private final double PRIME_UPTAKE_SPEAKER_SHOT_INTAKE_REVERSE_TIMEOUT = 0.1;
  private final double PRIME_UPTAKE_SPEAKER_SHOT_UPTAKE_SPEED = -1.0;

  private final double UPTAKE_SHOT_INTAKE_RELEASE_SPEED = -1.0;
  private final double UPTAKE_SHOT_UPTAKE_RELEASE_SPEED = -1.0;
  private final double UPTAKE_SHOT_INTAKE_UPTAKE_RELEASE_TIMEOUT = 1.0;

  private final double PARTY_MODE_SHOOTER_SPEED = 0.8;
  private final double PARTY_MODE_INTAKE_SPEED = -0.6;
  private final double PARTY_MODE_UPTAKE_SPEED = -0.6;
  private final double PARTY_MODE_INTAKE_REVERSE_TIMEOUT = 1.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register Named Commands //
    NamedCommands.registerCommand("MoveUptake0", new MoveUptake(m_uptakeSubsystem, () -> UPTAKE_STOP));
    NamedCommands.registerCommand("MoveUptake100", new MoveUptake(m_uptakeSubsystem, () -> -0.7));   
    NamedCommands.registerCommand("MoveUptake90", new MoveUptake(m_uptakeSubsystem, () -> -0.65));
    NamedCommands.registerCommand("MoveIntake0", new MoveIntake(m_intakeSubsystem, () -> INTAKE_STOP));
    NamedCommands.registerCommand("MoveIntake100", new MoveIntake(m_intakeSubsystem, () -> INTAKE_FULL_SPEED));
    NamedCommands.registerCommand("MoveIntake90", new MoveIntake(m_intakeSubsystem, () -> -0.7));

    NamedCommands.registerCommand(
      "MoveIntakeUntilNoteDetected",
      new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> INTAKE_UNTIL_NOTE_DETECTED_SPEED)
              .andThen(new MoveIntake(m_intakeSubsystem, () -> INTAKE_STOP).withTimeout(INTAKE_UNTIL_NOTE_DETECTED_TIMEOUT))
              .andThen(Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.ORANGE))));
    NamedCommands.registerCommand(
      "UptakeShoot",
      new MoveUptake(m_uptakeSubsystem, () -> UPTAKE_SHOOT_SPEED).withTimeout(UPTAKE_SHOOT_TIMEOUT)
      .andThen(new MoveIntake(m_intakeSubsystem, () -> UPTAKE_SHOOT_INTAKE_SPEED)));

    // Init Auto Chooser //
    autoChooser = AutoBuilder.buildAutoChooser("UptakeShoot");
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

    //#region Xbox Controller Bindings

    // Set default teleop command to drive //
    m_swerveDriveSubsystem.setDefaultCommand(
      m_swerveDriveSubsystem.drive(
        () -> -m_driverController.getLeftY(),   // Translation
        () -> -m_driverController.getLeftX(),   // Strafe
        () -> -m_driverController.getRightX(),
        //  + (m_limelight3Subsystem.alignHorizontal(LimelightConstants.HORIZONTAL_KP)
        //    * m_driverControllerRaw.getRawAxis(JoystickConstants.TRIGGER_RIGHT)),  // Rotation
        () -> m_driverController.getHID().getRightBumper()  // Half-Speed
      )
    );

    // If button 1 is held, then we lock onto amp heading
    // If button 2 is held, then we lock onto shoot heading
    // Otherwise, simple swerve


    // m_swerveDriveSubsystem.setDefaultCommand(
    //   m_swerveDriveSubsystem.driveHeading(
    //     () -> -m_driverController.getLeftY(),   // Translation
    //     () -> -m_driverController.getLeftX(),   // Strafe
    //     () -> -m_driverController.getRightX(),  // X component of angle
    //     () -> -m_driverController.getRightY()   // Y component of angle
    //   )
    // );

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST){
      // m_climberSubsystem.setDefaultCommand(new TestClimber(m_climberSubsystem));
      m_intakeSubsystem.setDefaultCommand(new TestIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
      m_uptakeSubsystem.setDefaultCommand(new TestUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
      m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.setArmSpeed(0.0), m_armSubsystem));
      m_shooterSubsystem.setDefaultCommand(new TestShooter(m_shooterSubsystem, () -> -m_operatorController.getRightTriggerAxis(), () -> -m_operatorController.getLeftTriggerAxis()));

      m_operatorController.a()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

      m_operatorController.b()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));

    } else {

      // Set Default Command for Intake
      m_intakeSubsystem.setDefaultCommand(
        new MoveIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
        
      // Set Default Command for Uptake
      m_uptakeSubsystem.setDefaultCommand(
        new MoveUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
          
      // Set Default Command for Arm
      // m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.stopArm(), m_armSubsystem));
      m_armSubsystem.setDefaultCommand(new HoldArmToPosition(m_armSubsystem));

      m_operatorController.povDown().whileTrue(
        Commands.run(() -> m_armSubsystem.setArmSpeed(MANUAL_ARM_DOWN_SPEED), m_armSubsystem));

      m_operatorController.povUp().whileTrue(
        Commands.run(() -> m_armSubsystem.setArmSpeed(MANUAL_ARM_UP_SPEED), m_armSubsystem));

      // Set Default Command for Shooter
      m_shooterSubsystem.setDefaultCommand(
        new TestShooter(
            m_shooterSubsystem,
            () -> -m_operatorController.getRightTriggerAxis(),
            () -> -m_operatorController.getLeftTriggerAxis()));

      // Use the "Left Bumper" button to lock onto amp heading //
      m_driverController.leftBumper()
      .onTrue(
        Commands.runOnce(() -> {
          m_swerveDriveSubsystem.setDefaultCommand(
            m_swerveDriveSubsystem.driveHeading(
            () -> -m_driverController.getLeftY(),   // Translation
            () -> -m_driverController.getLeftX(),   // Strafe
            () -> 90.0
          ));
        }))
      .onFalse(
        Commands.runOnce(() -> {
          m_swerveDriveSubsystem.setDefaultCommand(
            m_swerveDriveSubsystem.drive(
              () -> -m_driverController.getLeftY(),   // Translation
              () -> -m_driverController.getLeftX(),   // Strafe
              () -> -m_driverController.getRightX(),
              //  + (m_limelight3Subsystem.alignHorizontal(LimelightConstants.HORIZONTAL_KP)
              //    * m_driverControllerRaw.getRawAxis(JoystickConstants.TRIGGER_RIGHT)),  // Rotation
              () -> m_driverController.getHID().getRightBumper()  // Half-Speed
            )
          );
        }));

      // Use the "Right Bumper" button to lock onto shoot heading //
      m_driverController.rightBumper()
      .onTrue(
        Commands.run(() -> {
          m_swerveDriveSubsystem.setDefaultCommand(
            m_swerveDriveSubsystem.driveHeading(
            () -> -m_driverController.getLeftY(),   // Translation
            () -> -m_driverController.getLeftX(),   // Strafe
            () -> 0.0
          ));
        }))
      .onFalse(
        Commands.run(() -> {
          m_swerveDriveSubsystem.setDefaultCommand(
            m_swerveDriveSubsystem.drive(
              () -> -m_driverController.getLeftY(),   // Translation
              () -> -m_driverController.getLeftX(),   // Strafe
              () -> -m_driverController.getRightX(),
              //  + (m_limelight3Subsystem.alignHorizontal(LimelightConstants.HORIZONTAL_KP)
              //    * m_driverControllerRaw.getRawAxis(JoystickConstants.TRIGGER_RIGHT)),  // Rotation
              () -> m_driverController.getHID().getRightBumper()  // Half-Speed
            )
          );
        }));
      // Unjam //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_1)
          .whileTrue(new MoveIntake(m_intakeSubsystem, () -> INTAKE_UNJAM_SPEED));

      // Intake using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_2)
          .whileTrue(
              new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> INTAKE_UNTIL_NOTE_DETECTED_SPEED)
              .andThen(new MoveIntake(m_intakeSubsystem, () -> INTAKE_SLIGHT_REVERSE_SPEED).withTimeout(INTAKE_UNTIL_NOTE_DETECTED_TIMEOUT))
              .andThen(Commands.runOnce(() -> {
                m_ledSubsystem.set(LEDConstants.ORANGE);
                noteIsInIntakeEntry.setBoolean(true);
              }))
          );

      // Score Note to amp from intake using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_3)
          .onTrue(
              new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
              .andThen(Commands.runOnce(() -> m_shooterSubsystem.setIdleMode(IdleMode.kBrake)))
              .andThen(new WaitCommand(AMP_PREP_ARM_SETPOINT_WAIT_TIME))
              .andThen(
                  new MoveIntakeUptakeUntilNoteDetected(
                      m_intakeSubsystem, 
                      m_uptakeSubsystem, 
                      () -> AMP_PREP_INTAKE_VOLTAGE,
                      () -> AMP_PREP_UPTAKE_VOLTAGE)
                  .deadlineWith(new MoveShooter(m_shooterSubsystem, () -> AMP_PREP_SHOOTER_SPEED))
              )
              // .andThen(
              //   new MoveIntakeUptakeVoltage(
              //     m_intakeSubsystem, 
              //     m_uptakeSubsystem,
              //     () -> AMP_PREP_INTAKE_VOLTAGE,
              //     () -> AMP_PREP_UPTAKE_BUMP_VOLTAGE
              //   )
              //   .alongWith(new MoveShooter(m_shooterSubsystem, () -> AMP_PREP_UPTAKE_BUMP_VOLTAGE)).withTimeout(AMP_PREP_BUMP_TIMEOUT)
              // )
              .andThen(
                Commands.runOnce(() -> m_intakeSubsystem.set(INTAKE_STOP)),
                Commands.runOnce(() -> m_uptakeSubsystem.set(UPTAKE_STOP)),
                Commands.runOnce(() -> m_shooterSubsystem.set(SHOOTER_STOP)
              ))
          );
        
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_5)
      .onTrue(
          new MoveArmToPos(m_armSubsystem, ArmConstants.AMP_GOAL)
          .andThen(new MoveShooter(m_shooterSubsystem, () -> AMP_SCORE_SHOOTER_SPEED).withTimeout(AMP_SCORE_SHOOTER_TIMEOUT))
          .andThen(Commands.runOnce(() -> m_shooterSubsystem.setIdleMode(IdleMode.kCoast)))
            .andThen(new MoveArmToPos(m_armSubsystem, ArmConstants.AMP_ASSIST_GOAL))
            .andThen(
              Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.GREEN))
            )
            .andThen(new MoveShooter(m_shooterSubsystem, () -> SHOOTER_STOP).withTimeout(AMP_SCORE_SHOOTER_STOP_TIMEOUT))
            .andThen(new MoveArmToPos(m_armSubsystem, ArmConstants.INITIAL_GOAL))
            .andThen(Commands.runOnce(() -> {
              m_ledSubsystem.set(LEDConstants.BLACK);
              noteIsInIntakeEntry.setBoolean(false);
            }))

      );

      // Ferry Note using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_4)
          .onTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
            .andThen(
              new HoldArmToPosition(m_armSubsystem)
              .alongWith(
                Commands.runOnce(() -> m_shooterSubsystem.setIdleMode(IdleMode.kCoast)),
                new MoveShooter(m_shooterSubsystem, () -> FERRY_SHOOTER_SPEED)
              )
            )              
          ).onFalse(
            // Lock the robot at 10 degrees heading before ferrying the note //
            new MoveIntakeUptake(
              m_intakeSubsystem,
              m_uptakeSubsystem,
              () -> FERRY_INTAKE_RELEASE_SPEED,
              () -> FERRY_UPTAKE_RELEASE_SPEED)
            .withTimeout(FERRY_INTAKE_UPTAKE_RELEASE_TIMEOUT)
            .andThen(new MoveShooter(m_shooterSubsystem, () -> SHOOTER_STOP).withTimeout(FERRY_SHOOTER_STOP_TIMEOUT))
            .andThen(Commands.runOnce(() -> {
              m_ledSubsystem.set(LEDConstants.BLACK);
              noteIsInIntakeEntry.setBoolean(false);
            }))
            .andThen(
              Commands.runOnce(() -> m_intakeSubsystem.set(INTAKE_STOP)),
              Commands.runOnce(() -> m_uptakeSubsystem.set(UPTAKE_STOP)),
              Commands.runOnce(() -> m_shooterSubsystem.set(SHOOTER_STOP)
            ))
          );

      // Prime Uptake Shot //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_11)
        .whileTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.INITIAL_GOAL)
            .andThen(new MoveIntake(m_intakeSubsystem, () -> PRIME_UPTAKE_SPEAKER_SHOT_INTAKE_REVERSE_SPEED).withTimeout(PRIME_UPTAKE_SPEAKER_SHOT_INTAKE_REVERSE_TIMEOUT))
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.set(INTAKE_STOP)))
            .andThen(new MoveUptake(m_uptakeSubsystem, () -> PRIME_UPTAKE_SPEAKER_SHOT_UPTAKE_SPEED))
        ).whileFalse(
          new MoveIntakeUptake(m_intakeSubsystem, m_uptakeSubsystem, () -> UPTAKE_SHOT_INTAKE_RELEASE_SPEED, () -> UPTAKE_SHOT_UPTAKE_RELEASE_SPEED).withTimeout(UPTAKE_SHOT_INTAKE_UPTAKE_RELEASE_TIMEOUT)
          .andThen(Commands.runOnce(() -> {
            m_ledSubsystem.set(LEDConstants.BLACK);
            noteIsInIntakeEntry.setBoolean(false);
          }))
        );

      // Party Mode - AKA Trick Shot //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_12)
        .whileTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
            .andThen(
              new HoldArmToPosition(m_armSubsystem)
              .alongWith(
                new MoveShooter(m_shooterSubsystem, () -> PARTY_MODE_SHOOTER_SPEED)
              )
            )              
          ).whileFalse(
            new MoveIntakeUptake(m_intakeSubsystem, m_uptakeSubsystem, () -> PARTY_MODE_INTAKE_SPEED, () -> PARTY_MODE_UPTAKE_SPEED).withTimeout(PARTY_MODE_INTAKE_REVERSE_TIMEOUT)
            .andThen(Commands.runOnce(() -> m_shooterSubsystem.set(SHOOTER_STOP)))
            .andThen(Commands.runOnce(() -> {
                m_ledSubsystem.set(LEDConstants.BLACK);
                noteIsInIntakeEntry.setBoolean(false);
              }))
          );
    }

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.zeroGyro()));

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.lock()));

    // Test Lock Heading //
    var alliance = DriverStation.getAlliance();
    boolean color = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    double desiredFerryAngle = color ? 10.0 : -10.0;
    m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_7)
        .onTrue(
          Commands.run(
            () -> m_swerveDriveSubsystem.swerve.drive(
              m_swerveDriveSubsystem.swerve.swerveController.getTargetSpeeds(
                0,
                0,
                desiredFerryAngle,
                m_swerveDriveSubsystem.swerve.getYaw().getRadians(),
                m_swerveDriveSubsystem.swerve.swerveController.config.maxAngularVelocity
              )
            ),
            m_swerveDriveSubsystem
          ).until(() -> Math.abs(m_swerveDriveSubsystem.swerve.getYaw().getDegrees()) <= 0.1)
        );

    // m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_7)
    //   .onTrue(
    //     new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.7)
    //     .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.4).withTimeout(0.2)
    //     .raceWith(
    //         new MoveUptake(m_uptakeSubsystem, () -> 0.4)
    //     )
    //     .andThen(Commands.runOnce(() -> {
    //               m_intakeSubsystem.set(0);
    //               m_uptakeSubsystem.set(0);
    //             }))
    //     .andThen(
    //       new MoveUptake(m_uptakeSubsystem, () -> -1.0).withTimeout(1.0))
    //       .andThen(new MoveIntake(m_intakeSubsystem, () -> -0.5)))
    //   );
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
