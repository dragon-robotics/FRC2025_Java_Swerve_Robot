// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

// import org.littletonrobotics.junction.LogFileUtil;
// import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGReader;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// import com.pathplanner.lib.pathfinding.Pathfinding;

// import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the LoggedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
// public class Robot extends LoggedRobot {
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Commented out as a last resort test to solve our amp sequence issues
  // public Robot(){
  //   super(0.01);
  // }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // // PathFinder and AdvantageKit Compatibility //
    // Pathfinding.setPathfinder(new LocalADStarAK());

    // // Instantiate our AdvantageKit Logger //

    // // Record metadata
    // Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // // Check if our git build is dirty or not //
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     Logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     Logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     Logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }

    // // Set up data receivers & replay source
    // switch (Constants.currentMode) {
    //   case REAL:
    //     // Running on a real robot, log to a USB stick ("/U/logs")
    //     Logger.addDataReceiver(new WPILOGWriter());
    //     Logger.addDataReceiver(new NT4Publisher());
    //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    //     break;
    //   case SIM:
    //     // Running a physics simulator, log to NT
    //     Logger.addDataReceiver(new NT4Publisher());
    //     break;
    //   case REPLAY:
    //     // Replaying a log, set up replay source
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog();
    //     Logger.setReplaySource(new WPILOGReader(logPath));
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    //     break;
    // }

    // // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // Warmup PathPlanner to avoid Java pauses
    PathfindingCommand.warmupCommand().schedule();
    FollowPathCommand.warmupCommand().schedule();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Lock Robot Pose //
    // m_robotContainer.m_swerveDriveSubsystem.lock();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // The swerve drive should be in brake mode during auto to improve accuracy //
    m_robotContainer.m_swerveDriveSubsystem.configNeutralMode(NeutralModeValue.Brake);

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
   
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_robotContainer.m_superstructureSubsystem.initCurrentHeading();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // m_robotContainer.m_superstructureSubsystem.initCurrentHeading();
  }

  @Override
  public void teleopInit() {

    // The swerve drive should be in coast mode during teleop for speed //
    m_robotContainer.m_swerveDriveSubsystem.configNeutralMode(NeutralModeValue.Coast);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Initialize current heading after auto
    m_robotContainer.m_superstructureSubsystem.initCurrentHeading();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
