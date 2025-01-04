// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;

public class LEDSubsystem extends SubsystemBase {

  // LED Controller //
  private final Spark m_ledController = new Spark(ShooterConstants.LED_CHANNEL);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // Initialize LED to black
    m_ledController.set(LEDConstants.BLACK);
  }

  public void set(double color) {
    m_ledController.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
