// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreAmp(
    IntakeSubsystem intake,
    UptakeSubsystem uptake,
    ArmSubsystem arm,
    ShooterSubsystem shooter,
    LEDSubsystem led
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmToPos(arm, ArmConstants.SHOOTER_GOAL),
      new WaitCommand(0.5),
      new MoveIntakeUptakeUntilNoteDetected(
          intake,
          uptake, 
          () -> - 0.6, 
          () -> -0.4)
      .deadlineWith(new MoveShooter(shooter, () -> -0.2)),
      Commands.runOnce(() -> shooter.set(0.0)),
      new MoveArmToPos(arm, ArmConstants.AMP_GOAL),
      new WaitCommand(0.5),
      new MoveShooter(shooter, () -> -0.5).withTimeout(0.5),
      new MoveArmToPos(arm, ArmConstants.AMP_ASSIST_GOAL),
      new MoveArmToPos(arm, ArmConstants.INITIAL_GOAL).deadlineWith(
        Commands.runOnce(() -> led.set(LEDConstants.GREEN)),
        Commands.runOnce(() -> shooter.set(0.0))        
      ),
      Commands.runOnce(() -> led.set(LEDConstants.BLACK))
    );
  }
}
