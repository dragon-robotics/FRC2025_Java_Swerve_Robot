// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain m_swerve;
  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputs[] m_inputs;
  private final Alert[] m_disconnectedAlerts;

  // Per-Camera Logging
  private List<StructArrayPublisher<Pose3d>> m_tagPosesPerCamArrayPublisherList;
  private List<StructArrayPublisher<Pose3d>> m_robotPosesPerCamArrayPublisherList;
  private List<StructArrayPublisher<Pose3d>> m_robotPosesAcceptedPerCamArrayPublisherList;
  private List<StructArrayPublisher<Pose3d>> m_robotPosesRejectedPerCamArrayPublisherList;

  // Summary (All Camera) Logging

  private StructArrayPublisher<Pose3d> m_tagPosesArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesAcceptedArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesRejectedArrayPublisher;

  // Check for odometry initialization and use about  //
  private int m_stablePoseCounter = 5;
  private boolean m_odometryInitialized = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(
    CommandSwerveDrivetrain swerve,
    VisionConsumer consumer,
    VisionIO... io) {
    m_swerve = swerve;
    m_consumer = consumer;
    m_io = io;

    // Initialize the the inputs
    m_inputs = new VisionIOInputs[m_io.length];
    m_disconnectedAlerts = new Alert[m_io.length];

    // Initialize the per-camera loggers
    m_tagPosesPerCamArrayPublisherList = new LinkedList<>();
    m_robotPosesPerCamArrayPublisherList = new LinkedList<>();
    m_robotPosesAcceptedPerCamArrayPublisherList = new LinkedList<>();
    m_robotPosesRejectedPerCamArrayPublisherList = new LinkedList<>();

    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputs();
      m_disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);
      
      // Initialize the per-camera loggers
      m_tagPosesPerCamArrayPublisherList.add(
          NetworkTableInstance.getDefault().getStructArrayTopic(
              "Vision/Camera-" + io[i].getCameraName() + "/TagPoses", Pose3d.struct).publish());
      m_robotPosesPerCamArrayPublisherList.add(
          NetworkTableInstance.getDefault().getStructArrayTopic(
              "Vision/Camera-" + io[i].getCameraName() + "/RobotPoses", Pose3d.struct).publish());
      m_robotPosesAcceptedPerCamArrayPublisherList.add(
          NetworkTableInstance.getDefault().getStructArrayTopic(
              "Vision/Camera-" + io[i].getCameraName() + "/RobotPosesAccepted", Pose3d.struct).publish());
      m_robotPosesRejectedPerCamArrayPublisherList.add(
          NetworkTableInstance.getDefault().getStructArrayTopic(
              "Vision/Camera-" + io[i].getCameraName() + "/RobotPosesRejected", Pose3d.struct).publish());
    }

    // Initialize the summary loggers
    m_tagPosesArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Summary/TagPoses", Pose3d.struct).publish();
    m_robotPosesArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Summary/RobotPoses", Pose3d.struct).publish();
    m_robotPosesAcceptedArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Summary/RobotPosesAccepted", Pose3d.struct).publish();
    m_robotPosesRejectedArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Summary/RobotPosesRejected", Pose3d.struct).publish();
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < m_io.length; i++) {
      m_io[i].updateInputs(m_inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < m_io.length; cameraIndex++) {
      // Update disconnected alert
      m_disconnectedAlerts[cameraIndex].set(!m_inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : m_inputs[cameraIndex].tagIds) {
        var tagPose = APTAG_FIELD_LAYOUT.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : m_inputs[cameraIndex].poseObservations) {

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > APTAG_FIELD_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > APTAG_FIELD_LAYOUT.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          m_stablePoseCounter = 5; // Reset counter if we reject a pose
          robotPosesRejected.add(observation.pose());
        } else {

          // If odometry isn't initialized, check we have 5 consecutive good poses
          if (!m_odometryInitialized) {
            if (m_stablePoseCounter > 0) {
              m_stablePoseCounter--;
            } else {
              // If we have 5 good poses, set odometry
              m_swerve.resetPose(observation.pose().toPose2d());
              m_odometryInitialized = true;
            }
          }

          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
          (1 + observation.averageTagDistance()) * (1 + observation.ambiguity()) /
          Math.sqrt(Math.max(observation.tagCount(), 1));

        double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= LINEAR_STDDEV_MEGATAG2_FACTOR;
          angularStdDev *= ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR;
        }
        if (cameraIndex < CAMERA_STDDEV_FACTORS.length) {
          // --- Calculate Dynamic Per-Camera/Observation Factor ---
          // Start with a base factor of 1.0
          double dynamicCamFactor = 1.0;

          // Increase factor based on latency: Higher latency = less trustworthy
          // Example: Add 1.0 to the factor for every 100ms of latency. Adjust the multiplier (10.0) as needed.
          dynamicCamFactor += observation.latencySeconds() * 10.0;

          // Increase factor based on edge proximity: Higher edgeFactor = less trustworthy
          // Since edgeFactor is already >= 1.0, we can multiply by it directly,
          // or use a function of it if we want a different scaling.
          // Example: Multiply by edgeFactor. If edgeFactor is 1.5, uncertainty increases by 50%.
          dynamicCamFactor *= observation.edgeFactor();

          // Apply the calculated dynamic factor
          linearStdDev *= dynamicCamFactor;
          angularStdDev *= dynamicCamFactor;
        }

        // System.out.println("Linear StdDev: " + linearStdDev);
        // System.out.println("Angular StdDev: " + angularStdDev);

        // Send vision observation
        m_consumer.accept(
            observation.pose().toPose2d(),
            Utils.fpgaToCurrentTime(observation.timestamp()),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      m_tagPosesPerCamArrayPublisherList.get(cameraIndex).set(
        tagPoses.toArray(new Pose3d[tagPoses.size()]));
      m_robotPosesPerCamArrayPublisherList.get(cameraIndex).set(
        robotPoses.toArray(new Pose3d[robotPoses.size()]));
      m_robotPosesAcceptedPerCamArrayPublisherList.get(cameraIndex).set(
        robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      m_robotPosesRejectedPerCamArrayPublisherList.get(cameraIndex).set(
        robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
  
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    m_tagPosesArrayPublisher.set(
      allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesArrayPublisher.set(
      allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    m_robotPosesAcceptedArrayPublisher.set(
      allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    m_robotPosesRejectedArrayPublisher.set(
      allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}
