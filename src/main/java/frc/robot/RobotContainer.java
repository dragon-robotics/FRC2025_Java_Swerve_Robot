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
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;

import com.pathplanner.lib.auto.AutoBuilder;
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

    // Swerve Drive Default Command //
    m_swerveDriveSubsystem.setDefaultCommand(
        m_superstructureSubsystem.DefaultDriveCommand(
          () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX(),
          () -> -m_driverController.getRightX(),
          () -> m_driverController.getHID().getXButton())
      );

    // Coral Subsystem Default Command //
    m_coralSubsystem.setDefaultCommand(
        new InstantCommand(() -> m_coralSubsystem.setCoralState(CoralSubsystem.CoralState.IDLE))
            .andThen(
                new MoveCoralManually(
                    m_coralSubsystem,
                    () -> (m_operatorController.getLeftTriggerAxis() - m_operatorController.getRightTriggerAxis()))));

    // Algae Subsystem Default Command //
    m_algaeSubsystem.setDefaultCommand(
        new InstantCommand(() -> m_algaeSubsystem.setAlgaeState(AlgaeSubsystem.AlgaeState.IDLE))
            .andThen(
                    new MoveAlgaeManually(
                        m_algaeSubsystem,
                        () -> m_operatorController.getLeftY(),
                        () -> m_operatorController.getRightY())));

    // Elevator Subsystem Default Commands //
    m_operatorController.povUp().whileTrue(
        new InstantCommand(() -> m_elevatorSubsystem.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE))
            .andThen(new InstantCommand(() -> m_elevatorSubsystem.setElevatorMotorSpeed(-0.2))))
            .onFalse(new InstantCommand(() -> m_elevatorSubsystem.setElevatorMotorSpeed(0)));

    m_operatorController.povDown().whileTrue(
        new InstantCommand(() -> m_elevatorSubsystem.setElevatorState(ElevatorSubsystem.ElevatorState.IDLE))
            .andThen(new InstantCommand(() -> m_elevatorSubsystem.setElevatorMotorSpeed(0.2))))
            .onFalse(new InstantCommand(() -> m_elevatorSubsystem.setElevatorMotorSpeed(0)));

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(m_superstructureSubsystem.SeedFieldCentric());

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().whileTrue(m_superstructureSubsystem.SwerveBrake());

    // Press the left bumper to trigger coral intake //
    m_driverController.leftBumper()
        .onTrue(m_superstructureSubsystem.IntakeCoral());

    m_driverController.rightBumper()
        .whileTrue(m_superstructureSubsystem.AimAndRangeApriltag());

    // Operator button box controls //
    
    // Test Controls //

    // Test the elevator //
    // L1 - Move to L1
    m_operatorButtonBoxController.button(1)
        .onTrue(m_superstructureSubsystem.ElevatorHome());

    m_operatorButtonBoxController.button(2)
        .onTrue(m_superstructureSubsystem.ElevatorL1());

    m_operatorButtonBoxController.button(3)
        .onTrue(m_superstructureSubsystem.ElevatorL2());

    m_operatorButtonBoxController.button(4)
        .onTrue(m_superstructureSubsystem.ElevatorL3());

    m_operatorButtonBoxController.button(5)
        .onTrue(m_superstructureSubsystem.ElevatorL4());

    // Test the coral intake //

    // Test the algae arm and intake //
    m_operatorButtonBoxController.button(6)
        .whileTrue(m_superstructureSubsystem.AlgaeArmHome());

    m_operatorButtonBoxController.button(7)
        .whileTrue(m_superstructureSubsystem.AlgaeArmIntake());

    m_operatorButtonBoxController.button(8)
        .whileTrue(m_superstructureSubsystem.AlgaeArmDeAlgaeify());

    m_operatorButtonBoxController.button(9)
        .whileTrue(m_superstructureSubsystem.AlgaeArmHold());

    m_operatorButtonBoxController.button(10)
        .whileTrue(m_superstructureSubsystem.AlgaeArmEject());
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
