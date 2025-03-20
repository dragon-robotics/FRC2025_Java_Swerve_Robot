// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.VisionConstants.DESIRED_RANGE;
import static frc.robot.Constants.VisionConstants.DESIRED_YAW_LEFT;
import static frc.robot.Constants.VisionConstants.DESIRED_YAW_RIGHT;

import java.util.Arrays;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.swerve_constant.TunerConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToReefTag extends Command {

  private VisionSubsystem m_vision;
  private CommandSwerveDrivetrain m_swerve;
  private boolean m_useLeftCamera;
  private PIDController m_aimController;
  private PIDController m_rangeController;
  private SwerveRequest.FieldCentricFacingAngle m_driveMaintainHeading;
  private double m_maxSpeed;
  private double m_tagRange;
  private double m_tagYaw;
  private boolean m_targetVisible;
  private int m_lockedTagId;

  /** Creates a new AutoAlignToReefTag. */
  public AutoAlignToReefTag(
    boolean useLeftCamera,
    PIDController aimController,
    PIDController rangeController,
    SwerveRequest.FieldCentricFacingAngle driveMaintainHeading,
    VisionSubsystem vision,
    CommandSwerveDrivetrain swerve
  ) {
    m_useLeftCamera = useLeftCamera;
    m_aimController = aimController;
    m_rangeController = rangeController;
    m_driveMaintainHeading = driveMaintainHeading;

    m_maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max speed at 12 volts
    m_tagRange = 999.999;
    m_tagYaw = 999.999;
    m_targetVisible = false;
    m_lockedTagId = -1;

    m_vision = vision;
    m_swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vision);
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // List<PhotonPipelineResult> results = m_vision.getCamera(m_useLeftCamera).getAllUnreadResults();
    // if (!results.isEmpty()) {
    //   PhotonPipelineResult result = results.get(results.size() - 1);
    //   if (result.hasTargets()) {
    //     int currentTagId = result.getBestTarget().getFiducialId();
    //     if (Arrays.stream(GeneralConstants.REEF_STATION_TAG_IDS).anyMatch(i -> i == currentTagId)) {
    //       // Get the best reef target and lock it in //
    //       m_lockedTagId = currentTagId;
    //     }
    //   }
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    List<PhotonPipelineResult> results = m_vision.getCamera(m_useLeftCamera).getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      PhotonPipelineResult result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // Get the current target
        PhotonTrackedTarget currentTag = result.getBestTarget();
        int currentTagId = result.getBestTarget().getFiducialId();
        
        if (Arrays.stream(GeneralConstants.REEF_STATION_TAG_IDS).anyMatch(i -> i == currentTagId)) {
          m_lockedTagId = currentTagId;

          // Found a reef station tag, record its information
          Transform3d cameraToTag = currentTag.getBestCameraToTarget();
          Translation3d tagTranslation = cameraToTag.getTranslation();

          m_tagRange = tagTranslation.getX();
          m_tagYaw = tagTranslation.getY();

          m_targetVisible = true;
        }
      }
    }

    // If the target is visible, aim and range to it
    if (m_targetVisible) {
      // Override the driver's turn and fwd/rev command with an automatic one
      // That turns strafe towards the tag, and gets the range right.

      // Calculate range error
      double rangeError = m_tagRange - DESIRED_RANGE; // 8.364 inches is the distance to the wall of the reef
      double forwardCorrection = m_rangeController.calculate(m_tagRange, DESIRED_RANGE);

      // Calculate yaw error
      double desiredYaw = m_useLeftCamera ? DESIRED_YAW_LEFT : DESIRED_YAW_RIGHT;
      double yawError = m_tagYaw - desiredYaw;
      double strafeCorrection = m_aimController.calculate(m_tagYaw, desiredYaw);

      double forward = -forwardCorrection;
      double strafe = -strafeCorrection;

      // Optionally clamp outputs to your robot’s maximum speed.
      forward = MathUtil.clamp(forward, -m_maxSpeed, m_maxSpeed);
      strafe  = MathUtil.clamp(strafe, -m_maxSpeed, m_maxSpeed);

      System.out.println(
          " Tag ID: " + m_lockedTagId +    
          " Strafe: " + Double.toString(strafe) +
          " Forward: " + Double.toString(forward) +
          " At Range Setpoint: " + Boolean.toString(m_rangeController.atSetpoint()) +
          " TagRange: " + Double.toString(m_tagRange) +
          " RangeError: " + Double.toString(rangeError) +
          " RangeCorrection: " + Double.toString(forwardCorrection) +
          " At Yaw Setpoint: " + Boolean.toString(m_aimController.atSetpoint()) +
          " TagYaw: " + Double.toString(m_tagYaw) +
          " YawError: " + Double.toString(yawError) +
          " YawCorrection: " + Double.toString(strafeCorrection));

      m_swerve.setOperatorPerspectiveForward(GeneralConstants.REEF_STATION_ID_ANGLE_MAP.get(m_lockedTagId));
      m_swerve.setControl(
          m_driveMaintainHeading
              .withVelocityX(forward)
              .withVelocityY(strafe)
              .withTargetDirection(Rotation2d.kZero));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset the swerve to operator perspective
    m_swerve.setOperatorPerspectiveForward(Rotation2d.kZero);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rangeController.atSetpoint() && m_aimController.atSetpoint();
  }
}
