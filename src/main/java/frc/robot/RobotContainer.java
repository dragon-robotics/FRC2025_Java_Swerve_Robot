// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.PhotonUtils;

import frc.robot.Constants.ElevatorSubsystemConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.commands.Teleop.IntakeAlgaeUntilAlgaeDetected;
import frc.robot.commands.Teleop.IntakeCoralUntilCoralDetected;
import frc.robot.commands.Teleop.MoveElevator;
import frc.robot.commands.Teleop.ScoreAlgae;
import frc.robot.commands.Teleop.ScoreCoral;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSparkMax;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralIOSparkMax;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public final CommandSwerveDrivetrain m_swerveDriveSubsystem;
  public final CoralSubsystem m_coralSubsystem;
  public final ElevatorSubsystem m_elevatorSubsystem;
  public final AlgaeSubsystem m_algaeSubsystem;
  public final VisionSubsystem m_visionSubsystem;
  public final Superstructure m_superstructureSubsystem;

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;
  private final CommandJoystick m_operatorButtonBoxController;
  private final CommandJoystick m_testerController;

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
    // // Register Named Commands //
    // NamedCommands.registerCommand("IntakeAlgaeUntilAlgaeDetected", new IntakeAlgaeUntilAlgaeDetected(m_algaeSubsystem));
    // NamedCommands.registerCommand("ScoreAlgae", new ScoreAlgae(m_algaeSubsystem));
    // NamedCommands.registerCommand("IntakeCoralUntilCoralDetected", new IntakeCoralUntilCoralDetected(m_coralSubsystem));
    // NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(m_coralSubsystem));

    // NamedCommands.registerCommand("MoveToL1", new MoveElevator(m_elevatorSubsystem, ElevatorSubsystemConstants.LVL_1));
    // NamedCommands.registerCommand("MoveToL2", new MoveElevator(m_elevatorSubsystem, ElevatorSubsystemConstants.LVL_2));
    // NamedCommands.registerCommand("MoveToL3", new MoveElevator(m_elevatorSubsystem, ElevatorSubsystemConstants.LVL_3));
    // NamedCommands.registerCommand("MoveToL4", new MoveElevator(m_elevatorSubsystem, ElevatorSubsystemConstants.LVL_4));

    // Instantiate the joysticks //
    m_driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    m_operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);
    m_operatorButtonBoxController = new CommandJoystick(OperatorConstants.OPERATOR_BUTTON_PORT);
    m_testerController = new CommandJoystick(OperatorConstants.TEST_PORT);

    // Instantiate all the subsystems //
    m_swerveDriveSubsystem = TunerConstants.createDrivetrain(
        250,
        SwerveConstants.ODOMETRY_STD,
        VisionConstants.DEFAULT_TAG_STDDEV);
    m_coralSubsystem = new CoralSubsystem(new CoralIOSparkMax());
    m_elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSparkMax());
    m_algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
    m_visionSubsystem = new VisionSubsystem(m_swerveDriveSubsystem.getState());

    // Create the superstructure subsystem //
    m_superstructureSubsystem = new Superstructure(
        m_swerveDriveSubsystem,
        m_coralSubsystem,
        m_elevatorSubsystem,
        m_algaeSubsystem,
        m_visionSubsystem,
        this);

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

    m_swerveDriveSubsystem.setDefaultCommand(
        m_superstructureSubsystem.DefaultDriveCommand(
          () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX(),
          () -> -m_driverController.getRightX(),
          () -> m_driverController.getHID().getRightBumperButton())
      );

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(m_superstructureSubsystem.SeedFieldCentric());

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().whileTrue(m_superstructureSubsystem.SwerveBrake());

    // Press the left bumper to trigger coral intake //
    m_driverController.leftBumper()
        .onTrue(
            m_superstructureSubsystem.setWantedSuperStateCommand(
                Superstructure.WantedSuperState.INTAKE_CORAL));

    // // Press the right bumper to align robot to tag + left/right offset + raise elevator
    // // Release the right bumper to score the coral and reset to default state //
    // m_driverController.rightBumper()
    //     .whileTrue(
    //         new ParallelCommandGroup(
    //             m_superstructureSubsystem.AimAndRangeApriltag(),
    //             m_superstructureSubsystem.setWantedSuperStateCommand(
    //                 Superstructure.WantedSuperState.ALIGN_TO_SCORE_CORAL)))
    //     .onFalse(
    //         m_superstructureSubsystem.setWantedSuperStateCommand(Superstructure.WantedSuperState.SCORE_CORAL)
    //         .andThen(new InstantCommand(() -> m_swerveDriveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero)));
    //         .andThen(new WaitCommand(0.5)) // Wait for the coral to be scored
    //         .andThen(m_superstructureSubsystem.setWantedSuperStateCommand(Superstructure.WantedSuperState.DEFAULT)));

    m_driverController.rightBumper()
        .whileTrue(m_superstructureSubsystem.AimAndRangeApriltag())
        .onFalse(new InstantCommand(() -> m_swerveDriveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero)));

    // Operator button box controls //
    
    // Test Controls //

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
