// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareToScoreSpeakerSequence extends SequentialCommandGroup {
  /** Creates a new PrepareToScoreSpeakerSequence. */
  public PrepareToScoreSpeakerSequence(
    SwerveDriveSubsystem swerve,
    UptakeSubsystem uptake,
    ArmSubsystem arm,
    ShooterSubsystem shooter
    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AimAtSpeaker(swerve),
      new MoveArmToShootPosition(arm),
      new MoveShooter(shooter, () -> 0.7)
    );
  }
}
