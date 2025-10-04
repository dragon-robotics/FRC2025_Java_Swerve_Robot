// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.GeneralConstants.*;
import static frc.robot.Constants.VisionConstants.APTAG_CAMERA_NAMES;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.led.LedIORevBlinkin;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain swerveSubsystem;
  public final CoralSubsystem coralSubsystem;
  public final ElevatorSubsystem elevatorSubsystem;
  public final AlgaeSubsystem algaeSubsystem;
  public final VisionSubsystem visionSubsystem;
  public final LedSubsystem ledSubsystem;
  public final Superstructure superstructureSubsystem;

  // Define Driver and Operator controllers //
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandJoystick operatorButtonBoxController;
  private final CommandJoystick testerController;

  private final SendableChooser<Command> autoChooser;

  // Swerve Commands //
  private Command defaultDriveCommand;
  private Command driveToClosestLeftReefPoseCommand;
  private Command driveToClosestRightReefPoseCommand;
  private Command driveToClosestCoralStationPoseCommand;
  private Command swerveBrakeCommand;
  private Command seedFieldCentricCommand;

  // Elevator Commands //
  private Command elevatorHomeCommand;
  private Command elevatorL1Command;
  private Command elevatorL2Command;
  private Command elevatorL3Command;
  private Command elevatorL4Command;
  private Command elevatorManualUpCommand;
  private Command elevatorManualDownCommand;
  private Command elevatorStopCommand;
  private Command elevatorSeedCommand;

  // Coral Commands //
  private Command slowReverseCoralIntakeCommand;
  private Command intakeCoralCommand;
  private Command intakeFromLeftCoralStationCommand;
  private Command intakeFromRightCoralStationCommand;
  private Command holdCoralCommand;
  private Command scoreCoralCommand;

  // Algae Commands //
  private Command algaeHomeCommand;
  private Command intakeAlgaeCommand;
  private Command holdAlgaeCommand;
  private Command scoreAlgaeCommand;
  private Command deAlgaeCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Add DogLog //
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
    DogLog.setPdh(new PowerDistribution());

    // Instantiate the joysticks //
    driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);
    operatorButtonBoxController = new CommandJoystick(OperatorConstants.OPERATOR_BUTTON_PORT);
    testerController = new CommandJoystick(OperatorConstants.TEST_PORT);

    // Instantiate all the subsystems //
    swerveSubsystem =
        TunerConstants.createDrivetrain(
            250, SwerveConstants.ODOMETRY_STD, VisionConstants.DEFAULT_TAG_STDDEV);

    // Change initialization based on the state of the robot //
    switch (CURRENT_MODE) {
      case COMP:
        coralSubsystem = new CoralSubsystem(new CoralIOSparkMax());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSparkMax());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        ledSubsystem = new LedSubsystem(new LedIORevBlinkin());
        break;
      case SIM:
        coralSubsystem = new CoralSubsystem(new CoralIOSparkMax());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSparkMax());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                // Auto-Align Cameras //
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVisionSim(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState)
                // // Apriltag Pose-Estimation Cameras //
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[2],
                //     VisionConstants.APTAG_POSE_EST_CAFL_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[3],
                //     VisionConstants.APTAG_POSE_EST_CAFR_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[4],
                //     VisionConstants.APTAG_POSE_EST_CABL_POS,
                //     swerveSubsystem::getState),
                // new VisionIOPhotonVisionSim(
                //     APTAG_CAMERA_NAMES[5],
                //     VisionConstants.APTAG_POSE_EST_CABR_POS,
                //     swerveSubsystem::getState)
                );
        ledSubsystem = new LedSubsystem(new LedIORevBlinkin());
        break;
      case TEST:
        coralSubsystem = new CoralSubsystem(new CoralIOSparkMax());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSparkMax());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        ledSubsystem = new LedSubsystem(new LedIORevBlinkin());
        break;
      default: // Default should be in comp mode //
        coralSubsystem = new CoralSubsystem(new CoralIOSparkMax());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSparkMax());
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
        visionSubsystem =
            new VisionSubsystem(
                swerveSubsystem,
                swerveSubsystem::addVisionMeasurement,
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[0],
                    VisionConstants.APTAG_ALIGN_LEFT_CAM_POS,
                    swerveSubsystem::getState),
                new VisionIOPhotonVision(
                    APTAG_CAMERA_NAMES[1],
                    VisionConstants.APTAG_ALIGN_RIGHT_CAM_POS,
                    swerveSubsystem::getState));
        ledSubsystem = new LedSubsystem(new LedIORevBlinkin());
        break;
    }

    // Create the superstructure subsystem //
    superstructureSubsystem =
        new Superstructure(
            swerveSubsystem,
            coralSubsystem,
            elevatorSubsystem,
            algaeSubsystem,
            visionSubsystem,
            this);

    // Instantiate all commands used //

    // Instantiate Swerve Commands //
    defaultDriveCommand =
        superstructureSubsystem.DefaultDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getHID().getPOV() == 0);
    driveToClosestLeftReefPoseCommand = superstructureSubsystem.DriveToClosestReefPoseCommand(true);
    driveToClosestRightReefPoseCommand =
        superstructureSubsystem.DriveToClosestReefPoseCommand(false);
    driveToClosestCoralStationPoseCommand =
        superstructureSubsystem.DriveToClosestCoralStationPoseCommand();
    swerveBrakeCommand = superstructureSubsystem.SwerveBrake();
    seedFieldCentricCommand = superstructureSubsystem.SeedFieldCentric();

    // Instantiate Elevator Commands //
    elevatorHomeCommand = superstructureSubsystem.ElevatorHome();
    elevatorL1Command = superstructureSubsystem.ElevatorL1();
    elevatorL2Command = superstructureSubsystem.ElevatorL2();
    elevatorL3Command = superstructureSubsystem.ElevatorL3();
    elevatorL4Command = superstructureSubsystem.ElevatorL4();
    elevatorManualUpCommand = superstructureSubsystem.ElevatorManualUp();
    elevatorManualDownCommand = superstructureSubsystem.ElevatorManualDown();
    elevatorStopCommand = superstructureSubsystem.ElevatorStop();
    elevatorSeedCommand = superstructureSubsystem.ElevatorZero();

    // Instantiate Coral Commands //
    slowReverseCoralIntakeCommand = superstructureSubsystem.ReverseCoralIntake();
    intakeCoralCommand = superstructureSubsystem.IntakeCoral();
    holdCoralCommand = superstructureSubsystem.HoldCoral();
    scoreCoralCommand = superstructureSubsystem.ScoreCoral();

    // Instantiate Algae Commands //
    algaeHomeCommand = superstructureSubsystem.AlgaeArmHome();
    intakeAlgaeCommand = superstructureSubsystem.IntakeAlgae();
    holdAlgaeCommand = superstructureSubsystem.AlgaeArmHold();
    scoreAlgaeCommand = superstructureSubsystem.ScoreAlgae();
    deAlgaeCommand = superstructureSubsystem.AlgaeArmDeAlgaeify();

    // Register Named Commands //
    // Register Swerve Commands //
    NamedCommands.registerCommand("DefaultDrive", defaultDriveCommand);
    NamedCommands.registerCommand("SwerveBrake", swerveBrakeCommand);
    NamedCommands.registerCommand("SeedFieldCentric", seedFieldCentricCommand);

    // Register Elevator Commands //
    NamedCommands.registerCommand("ElevatorHome", elevatorHomeCommand);
    NamedCommands.registerCommand("ElevatorL1", elevatorL1Command);
    NamedCommands.registerCommand("ElevatorL2", elevatorL2Command);
    NamedCommands.registerCommand("ElevatorL3", elevatorL3Command);
    NamedCommands.registerCommand("ElevatorL4", elevatorL4Command);
    NamedCommands.registerCommand("ElevatorManualUp", elevatorManualUpCommand);
    NamedCommands.registerCommand("ElevatorManualDown", elevatorManualDownCommand);
    NamedCommands.registerCommand("ElevatorStop", elevatorStopCommand);
    NamedCommands.registerCommand("ElevatorSeed", elevatorSeedCommand);

    // Register Coral Commands //
    NamedCommands.registerCommand("ReverseIntakeCoral", slowReverseCoralIntakeCommand);
    NamedCommands.registerCommand("IntakeCoral", intakeCoralCommand);
    NamedCommands.registerCommand("IntakeFromLeftCoralStation", intakeFromLeftCoralStationCommand);
    NamedCommands.registerCommand(
        "IntakeFromRightCoralStation", intakeFromRightCoralStationCommand);
    NamedCommands.registerCommand("HoldCoral", holdCoralCommand);
    NamedCommands.registerCommand("ScoreCoral", scoreCoralCommand);

    // Register Algae Commands //
    NamedCommands.registerCommand("AlgaeHome", algaeHomeCommand);
    NamedCommands.registerCommand("IntakeAlgae", intakeAlgaeCommand);
    NamedCommands.registerCommand("HoldAlgae", holdAlgaeCommand);
    NamedCommands.registerCommand("ScoreAlgae", scoreAlgaeCommand);
    NamedCommands.registerCommand("DeAlgae", deAlgaeCommand);

    // Init Auto Chooser //
    autoChooser = AutoBuilder.buildAutoChooser("S6-R02-R03-R04-SeqElev-Auto");
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
    swerveSubsystem.setDefaultCommand(defaultDriveCommand);

    coralSubsystem.setDefaultCommand(holdCoralCommand);
    algaeSubsystem.setDefaultCommand(algaeHomeCommand);

    // Use the "Back" button to reset the Gyro orientation //
    driverController.back().onTrue(seedFieldCentricCommand);
    // Use the "Start" button to x-lock the wheels //
    driverController.start().onTrue(swerveBrakeCommand);

    // Press the left bumper to trigger coral intake //
    driverController.leftBumper().onTrue(intakeCoralCommand);

    // Pres the right bumper to score the coral //
    driverController.rightBumper().whileTrue(scoreCoralCommand);

    // Press the left trigger to intake algae //
    driverController.leftTrigger(0.2).whileTrue(intakeAlgaeCommand).onFalse(holdAlgaeCommand);

    // Press the right trigger to score algae //
    driverController.rightTrigger(0.2).whileTrue(scoreAlgaeCommand);

    // pov 270 = align left branch
    driverController.pov(270).whileTrue(driveToClosestLeftReefPoseCommand);

    // pov 90 = align right branch
    driverController.pov(90).whileTrue(driveToClosestRightReefPoseCommand);

    // pov 180 = auto-align to nearest coral station
    driverController.pov(180).whileTrue(driveToClosestCoralStationPoseCommand);

    // Right stick button = Dealgae
    driverController.rightStick().whileTrue(deAlgaeCommand);

    // Left stick button = Elevator Home
    driverController.leftStick().whileTrue(slowReverseCoralIntakeCommand);

    // // A = L1, toggle = Home / L1
    // driverController.a().toggleOnTrue(
    //     Commands.either(
    //         // If currently at L1, go to home
    //         superstructureSubsystem.ElevatorHome(),
    //         // If not at L1, go to L1
    //         superstructureSubsystem.ElevatorL1(),
    //         // Condition: check if elevator is currently at L1
    //         () -> elevatorSubsystem.getElevatorState() == ElevatorSubsystem.ElevatorState.L1
    //     )
    // );

    // // X = L2, toggle = Home / L2
    // driverController.b().toggleOnTrue(
    //     Commands.either(
    //         // If currently at L2, go to home
    //         superstructureSubsystem.ElevatorHome(),
    //         // If not at L2, go to L2
    //         superstructureSubsystem.ElevatorL2(),
    //         // Condition: check if elevator is currently at L2
    //         () -> elevatorSubsystem.getElevatorState() == ElevatorSubsystem.ElevatorState.L2
    //     )
    // );

    // // B = L3, toggle = Home / L3
    // driverController.x().toggleOnTrue(
    //     Commands.either(
    //         // If currently at L3, go to home
    //         superstructureSubsystem.ElevatorHome(),
    //         // If not at L3, go to L3
    //         superstructureSubsystem.ElevatorL3(),
    //         // Condition: check if elevator is currently at L3
    //         () -> elevatorSubsystem.getElevatorState() == ElevatorSubsystem.ElevatorState.L3
    //     )
    // );

    // // Y = L4, toggle = Home / L4
    // driverController.y().toggleOnTrue(
    //     Commands.either(
    //         // If currently at L4, go to home
    //         superstructureSubsystem.ElevatorHome(),
    //         // If not at L4, go to L4
    //         superstructureSubsystem.ElevatorL4(),
    //         // Condition: check if elevator is currently at L4
    //         () -> elevatorSubsystem.getElevatorState() == ElevatorSubsystem.ElevatorState.L4
    //     )
    // );

    // driverController.pov(90)
    //     .whileTrue(elevatorManualUpCommand)
    //     .onFalse(elevatorStopCommand); // Elevator Manual Up
    // driverController.pov(270)
    //     .whileTrue(elevatorManualDownCommand)
    //     .onFalse(elevatorStopCommand); // Elevator Manual Down
    // driverController.pov(180)
    //     .onTrue(elevatorSeedCommand); // Elevator Rezero

    // Elevator Triggers //
    operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_HOME_BTN)
        .onTrue(elevatorHomeCommand);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L1_BTN)
        .onTrue(elevatorL1Command);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L2_BTN)
        .onTrue(elevatorL2Command);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L3_BTN)
        .onTrue(elevatorL3Command);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.ELEVATOR_L4_BTN)
        .onTrue(elevatorL4Command);

    // Algae Triggers //
    operatorButtonBoxController
        .button(OperatorControlNameConstants.ALGAE_HOME_BTN)
        .onTrue(algaeHomeCommand);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.INTAKE_ALGAE_BTN)
        .whileTrue(intakeAlgaeCommand)
        .onFalse(holdAlgaeCommand);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.SCORE_ALGAE_BTN)
        .onTrue(scoreAlgaeCommand);

    operatorButtonBoxController
        .button(OperatorControlNameConstants.DEALGAE_BTN)
        .whileTrue(deAlgaeCommand);
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
