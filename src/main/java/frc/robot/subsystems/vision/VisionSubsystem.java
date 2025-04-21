// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.LinkedList;
import java.util.List;
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
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {

  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputs[] m_inputs;
  private final Alert[] m_disconnectedAlerts;

  // Per-Camera Logging
  StructArrayPublisher<Pose3d> m_tagPosesPerCamArrayPublisher;
  StructArrayPublisher<Pose3d> m_robotPosesPerCamArrayPublisher;
  StructArrayPublisher<Pose3d> m_robotPosesAcceptedPerCamArrayPublisher;
  StructArrayPublisher<Pose3d> m_robotPosesRejectedPerCamArrayPublisher;

  // Summary (All Camera) Logging

  private StructArrayPublisher<Pose3d> m_tagPosesArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesAcceptedArrayPublisher;
  private StructArrayPublisher<Pose3d> m_robotPosesRejectedArrayPublisher;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
    m_consumer = consumer;
    m_io = io;

    // Initialize the the inputs
    m_inputs = new VisionIOInputs[m_io.length];
    m_disconnectedAlerts = new Alert[m_io.length];

    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputs();
      m_disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
      
      // Initialize the per-camera loggers
      m_tagPosesPerCamArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Camera" + Integer.toString(i) + "/TagPoses", Pose3d.struct).publish();
      m_robotPosesPerCamArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Camera" + Integer.toString(i) + "/RobotPoses", Pose3d.struct).publish();
      m_robotPosesAcceptedPerCamArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Camera" + Integer.toString(i) + "/RobotPosesAccepted", Pose3d.struct).publish();
      m_robotPosesRejectedPerCamArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "Vision/Camera" + Integer.toString(i) + "/RobotPosesRejected", Pose3d.struct).publish();
    }

    // Initialize the summary loggers
    m_tagPosesArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Summary/TagPoses", Pose3d.struct).publish();
    m_robotPosesArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Summary/RobotPoses", Pose3d.struct).publish();
    m_robotPosesAcceptedArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Summary/RobotPosesAccepted", Pose3d.struct).publish();
    m_robotPosesRejectedArrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Vision/Summary/RobotPosesRejected", Pose3d.struct).publish();
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
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= LINEAR_STDDEV_MEGATAG2_FACTOR;
          angularStdDev *= ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR;
        }
        if (cameraIndex < CAMERA_STDDEV_FACTORS.length) {
          linearStdDev *= CAMERA_STDDEV_FACTORS[cameraIndex];
          angularStdDev *= CAMERA_STDDEV_FACTORS[cameraIndex];
        }

        // Send vision observation
        m_consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      m_tagPosesPerCamArrayPublisher.set(allTagPoses.toArray(new Pose3d[tagPoses.size()]));
      m_robotPosesPerCamArrayPublisher.set(allRobotPoses.toArray(new Pose3d[robotPoses.size()]));
      m_robotPosesAcceptedPerCamArrayPublisher.set(allRobotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      m_robotPosesRejectedPerCamArrayPublisher.set(allRobotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
  
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    m_tagPosesArrayPublisher.set(allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesArrayPublisher.set(allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    m_robotPosesAcceptedArrayPublisher.set(allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    m_robotPosesRejectedArrayPublisher.set(allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}
