// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Teleop.MoveAlgaeManually;
import frc.robot.commands.Teleop.MoveCoralManually;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOSparkMax;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralIOSparkMax;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  // Swerve Commands //
  private Command m_defaultDriveCommand;
  private Command m_aimAndAlignToReefApriltagCommand;
  private Command m_swerveBrakeCommand;
  private Command m_seedFieldCentricCommand;

  // Elevator Commands //
  private Command m_elevatorHomeCommand;
  private Command m_elevatorL1Command;
  private Command m_elevatorL2Command;
  private Command m_elevatorL3Command;
  private Command m_elevatorL4Command;

  // Coral Commands //
  private Command m_intakeCoralCommand;
  private Command m_intakeFromLeftCoralStationCommand;
  private Command m_intakeFromRightCoralStationCommand;
  private Command m_holdCoralCommand;
  private Command m_ejectCoralCommand;
  private Command m_alignToLeftReefBranchCommand;
  private Command m_alignToRightReefBranchCommand;

  // Algae Commands //
  private Command m_intakeAlgaeCommand;
  private Command m_deAlgaeCommand;
  private Command m_algaeHomeCommand;
  private Command m_ejectAlgaeCommand;
  private Command m_holdAlgaeCommand;

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

    // Instantiate all commands used //

    // Instantiate Swerve Commands //
    m_defaultDriveCommand = m_superstructureSubsystem.DefaultDriveCommand(
        () -> -m_driverController.getLeftY() * 0.8,
        () -> -m_driverController.getLeftX() * 0.8,
        () -> -m_driverController.getRightX() * 0.8,
        () -> m_driverController.getHID().getXButton());
    m_aimAndAlignToReefApriltagCommand = m_superstructureSubsystem.AimAndRangeReefApriltag();
    m_swerveBrakeCommand = m_superstructureSubsystem.SwerveBrake();
    m_seedFieldCentricCommand = m_superstructureSubsystem.SeedFieldCentric();

    // Instantiate Elevator Commands //
    m_elevatorHomeCommand = m_superstructureSubsystem.ElevatorHome();
    m_elevatorL1Command = m_superstructureSubsystem.ElevatorL1();
    m_elevatorL2Command = m_superstructureSubsystem.ElevatorL2();
    m_elevatorL3Command = m_superstructureSubsystem.ElevatorL3();
    m_elevatorL4Command = m_superstructureSubsystem.ElevatorL4();

    // Instantiate Coral Commands //
    m_intakeCoralCommand = m_superstructureSubsystem.IntakeCoral();
    m_holdCoralCommand = m_superstructureSubsystem.HoldCoral();
    m_ejectCoralCommand = m_superstructureSubsystem.ScoreCoral();
    m_intakeFromLeftCoralStationCommand = m_superstructureSubsystem.SetCoralStation(true);
    m_intakeFromLeftCoralStationCommand = m_superstructureSubsystem.SetCoralStation(false);
    m_alignToLeftReefBranchCommand = m_superstructureSubsystem.SetReefAlignment(true);
    m_alignToRightReefBranchCommand = m_superstructureSubsystem.SetReefAlignment(false);

    // Instantiate Algae Commands //
    m_algaeHomeCommand = m_superstructureSubsystem.AlgaeArmHome();
    m_deAlgaeCommand = m_superstructureSubsystem.AlgaeArmDeAlgaeify();
    m_intakeAlgaeCommand = m_superstructureSubsystem.AlgaeArmHome();
    m_ejectAlgaeCommand = m_superstructureSubsystem.ScoreAlgae();
    m_holdAlgaeCommand = m_superstructureSubsystem.AlgaeArmHold();

    // Register Named Commands //
    NamedCommands.registerCommand("DeAlgae", m_deAlgaeCommand);

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

    // Swerve Drive Default Command //
    m_swerveDriveSubsystem.setDefaultCommand(m_defaultDriveCommand);

    m_coralSubsystem.setDefaultCommand(m_holdCoralCommand);    
    m_algaeSubsystem.setDefaultCommand(m_algaeHomeCommand);
    m_elevatorSubsystem.setDefaultCommand(m_elevatorHomeCommand);

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(m_seedFieldCentricCommand);

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().whileTrue(m_swerveBrakeCommand);

    // Press the left bumper to trigger coral intake //
    m_driverController.leftBumper()
        .onTrue(m_intakeCoralCommand);

    m_driverController.rightBumper()
        .whileTrue(m_aimAndAlignToReefApriltagCommand)
        .onFalse(m_ejectCoralCommand);

    // Operator button box controls //
    // Set to intake left or right
    m_operatorButtonBoxController.button(5)
        .onTrue(m_intakeFromLeftCoralStationCommand);

    m_operatorButtonBoxController.button(6)
        .onTrue(m_intakeFromRightCoralStationCommand);
    
    // Test Controls //

    // Test the elevator //
    // L1 - Move to L1
    m_operatorButtonBoxController.button(10)
        .onTrue(m_elevatorHomeCommand);

    m_operatorButtonBoxController.button(4)
        .onTrue(m_elevatorL1Command);

    m_operatorButtonBoxController.button(3)
        .onTrue(m_elevatorL2Command);

    m_operatorButtonBoxController.button(2)
        .onTrue(m_elevatorL3Command);

    m_operatorButtonBoxController.button(1)
        .onTrue(m_elevatorL4Command);

    m_operatorButtonBoxController.button(11)
        .onTrue(m_alignToLeftReefBranchCommand);

    m_operatorButtonBoxController.button(12)
        .onTrue(m_alignToRightReefBranchCommand);


    // Test the coral intake //

    // // Test the algae arm and intake //
    m_operatorButtonBoxController.button(10)
        .onTrue(m_algaeHomeCommand);

    m_operatorButtonBoxController.button(7)
        .onTrue(m_intakeAlgaeCommand);

    m_operatorButtonBoxController.button(8)
        .onTrue(m_deAlgaeCommand);

    m_operatorButtonBoxController.button(9)
        .onTrue(m_holdAlgaeCommand);

    m_operatorButtonBoxController.button(9)
        .onTrue(m_ejectAlgaeCommand);
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
