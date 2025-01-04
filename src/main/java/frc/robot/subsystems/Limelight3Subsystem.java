// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.SwerveConstants;

public class Limelight3Subsystem extends SubsystemBase {
  private SlewRateLimiter m_rotlimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

  /** Creates a new Limelight3Subsystem. */
  public Limelight3Subsystem() {}

  public double limelight_aim_note_proportional(double kp) {
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our
      // proportional control loop
      // if it is too high, the robot will oscillate around.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.

      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
      // rightmost edge of
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = (LimelightHelpers.getTX("limelight-note") * Math.PI / 180) * kp;

      // convert to radians per second for our drive method
      targetingAngularVelocity *= 2 * Math.PI;

      // invert since tx is positive when the target is to the right of the
      // crosshair
      targetingAngularVelocity *= -1.0;

      return targetingAngularVelocity;
  }

  /* USED FOR VERTICAL ALIGNMENT - UNTESTED /

    public double getTargetAngle(double kp) {
      double ty = (LimelightHelpers.getTY("limelight") * Math.PI / 180) * kp;
      double aprilTagHeight = 1.431925;
      double limelightHeight = 0.3;
      double limelightAngle = 10.0;
      double targetHeight = 2.0429855;

      double distance = (aprilTagHeight - limelightHeight) / Math.tan(Math.toRadians(ty + limelightAngle));
      double a = distance * Math.cos(Math.toRadians(ty + limelightAngle));
      return Math.toDegrees(Math.atan2((targetHeight - limelightHeight), a));

  }
*/
  public double alignHorizontal(double kp) {
      final var rot_limelight = limelight_aim_note_proportional(kp);
      double m_current = m_rotlimiter.calculate(rot_limelight);
      SmartDashboard.putNumber("limelight tX", LimelightHelpers.getTX("limelight-note"));
      SmartDashboard.putNumber("limelight rot mod [deg]", m_current * (180 / Math.PI));
      return m_current;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
