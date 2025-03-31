// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorControlNameConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.algae.AlgaeIOSparkMax;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralIOSparkMax;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;
import frc.robot.util.OperatorDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private Command m_alignToLeftReefBranchCommand;
  private Command m_alignToRightReefBranchCommand;
  private Command m_scoreCoralCommand;

  // Algae Commands //
  private Command m_algaeHomeCommand;
  private Command m_intakeAlgaeCommand;
  private Command m_holdAlgaeCommand;
  private Command m_scoreAlgaeCommand;
  private Command m_deAlgaeCommand;

  // Operator Dashboard //
  private OperatorDashboard m_operatorDashboard;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_operatorDashboard = new OperatorDashboard();

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
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
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
    m_intakeFromLeftCoralStationCommand = m_superstructureSubsystem.SetCoralStation(true);
    m_intakeFromRightCoralStationCommand = m_superstructureSubsystem.SetCoralStation(false);
    m_holdCoralCommand = m_superstructureSubsystem.HoldCoral();
    m_alignToLeftReefBranchCommand = m_superstructureSubsystem.SetReefAlignment(true);
    m_alignToRightReefBranchCommand = m_superstructureSubsystem.SetReefAlignment(false);
    m_scoreCoralCommand = m_superstructureSubsystem.ScoreCoral();

    // Instantiate Algae Commands //
    m_algaeHomeCommand = m_superstructureSubsystem.AlgaeArmHome();
    m_intakeAlgaeCommand = m_superstructureSubsystem.IntakeAlgae();
    m_holdAlgaeCommand = m_superstructureSubsystem.AlgaeArmHold();
    m_scoreAlgaeCommand = m_superstructureSubsystem.ScoreAlgae();
    m_deAlgaeCommand = m_superstructureSubsystem.AlgaeArmDeAlgaeify();

    // Register Named Commands //
    // Register Swerve Commands //
    NamedCommands.registerCommand("DefaultDrive", m_defaultDriveCommand);
    NamedCommands.registerCommand("AimAndAlignToReefAprilTag", m_aimAndAlignToReefApriltagCommand);
    NamedCommands.registerCommand("SwerveBrake", m_swerveBrakeCommand);
    NamedCommands.registerCommand("SeedFieldCentric", m_seedFieldCentricCommand);

    // Register Elevator Commands //
    NamedCommands.registerCommand("ElevatorHome", m_elevatorHomeCommand);
    NamedCommands.registerCommand("ElevatorL1", m_elevatorL1Command);
    NamedCommands.registerCommand("ElevatorL2", m_elevatorL2Command);
    NamedCommands.registerCommand("ElevatorL3", m_elevatorL3Command);
    NamedCommands.registerCommand("ElevatorL4", m_elevatorL4Command);

    // Register Coral Commands //
    NamedCommands.registerCommand("IntakeCoral", m_intakeCoralCommand);
    NamedCommands.registerCommand("IntakeFromLeftCoralStation", m_intakeFromLeftCoralStationCommand);
    NamedCommands.registerCommand("IntakeFromRightCoralStation", m_intakeFromRightCoralStationCommand);
    NamedCommands.registerCommand("HoldCoral", m_holdCoralCommand);
    NamedCommands.registerCommand("AlignToLeftReefBranch", m_alignToLeftReefBranchCommand);
    NamedCommands.registerCommand("AlignToRightReefBranch", m_alignToRightReefBranchCommand);
    NamedCommands.registerCommand("ScoreCoral", m_scoreCoralCommand);

    // Register Algae Commands //
    NamedCommands.registerCommand("AlgaeHome", m_algaeHomeCommand);
    NamedCommands.registerCommand("IntakeAlgae", m_intakeAlgaeCommand);
    NamedCommands.registerCommand("HoldAlgae", m_holdAlgaeCommand);
    NamedCommands.registerCommand("ScoreAlgae", m_scoreAlgaeCommand);
    NamedCommands.registerCommand("DeAlgae", m_deAlgaeCommand);

    // Init Auto Chooser //
    autoChooser = AutoBuilder.buildAutoChooser("LeaveAutoS3");
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
    // m_elevatorSubsystem.setDefaultCommand(m_elevatorHomeCommand);

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(m_seedFieldCentricCommand);

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().whileTrue(m_swerveBrakeCommand);

    // Press the left bumper to trigger coral intake //
    m_driverController.leftBumper()
        .onTrue(m_intakeCoralCommand);

    m_driverController.rightBumper()
        .whileTrue(m_scoreCoralCommand);
    
    m_driverController.pov(0).whileTrue(m_aimAndAlignToReefApriltagCommand);

    // Operator button box controls //
    // // Set to intake left or right
    // m_operatorButtonBoxController.button(5)
    //     .onTrue(m_intakeFromLeftCoralStationCommand);

    // m_operatorButtonBoxController.button(6)
    //     .onTrue(m_intakeFromRightCoralStationCommand);

    // Elevator Triggers //
    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_HOME_BTN)
        .onTrue(m_elevatorHomeCommand);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L1_BTN)
        .onTrue(m_elevatorL1Command);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L2_BTN)
        .onTrue(m_elevatorL2Command);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L3_BTN)
        .onTrue(m_elevatorL3Command);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L4_BTN)
        .onTrue(m_elevatorL4Command);

    // Coral Triggers //

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ALIGN_LEFT_REEF_BRANCH_BTN)
        .onTrue(m_alignToLeftReefBranchCommand);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ALIGN_RIGHT_REEF_BRANCH_BTN)
        .onTrue(m_alignToRightReefBranchCommand);

    // Algae Triggers //
    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.ALGAE_HOME_BTN)
        .onTrue(m_algaeHomeCommand);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.INTAKE_ALGAE_BTN)
        .whileTrue(m_intakeAlgaeCommand)
        .onFalse(m_holdAlgaeCommand);
        
    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.SCORE_ALGAE_BTN)
        .onTrue(m_scoreAlgaeCommand);

    m_operatorButtonBoxController
        .button(OperatorControlNameConstants.DEALGAE_BTN)
        .whileTrue(m_deAlgaeCommand);

    m_driverController
        .y()
        .whileTrue(m_deAlgaeCommand);
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
