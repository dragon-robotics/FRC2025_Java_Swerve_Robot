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

    // #region Xbox Controller Bindings

    // Set default teleop command to drive //
    m_swerveDriveSubsystem.setDefaultCommand(
      m_swerveDriveSubsystem.drive(
        () -> -m_driverController.getLeftY(),   // Translation
        () -> -m_driverController.getLeftX(),   // Strafe
        () -> -m_driverController.getRightX(),
        () -> m_driverController.getHID().getRightBumper()  // Half-Speed
      )
    );

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST){
      m_intakeSubsystem.setDefaultCommand(new TestIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
      m_uptakeSubsystem.setDefaultCommand(new TestUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
      m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.setArmSpeed(0.0), m_armSubsystem));
      m_shooterSubsystem.setDefaultCommand(new TestShooter(m_shooterSubsystem, () -> -m_operatorController.getRightTriggerAxis(), () -> -m_operatorController.getLeftTriggerAxis()));

      m_operatorController.a()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

      m_operatorController.b()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));

    }

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.zeroGyro()));

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.lock()));
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
