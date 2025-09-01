// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.vision.AcceptedPose;
import frc.robot.util.vision.RejectedPose;
import frc.robot.util.vision.VisionPoseValidator;
import java.util.LinkedList;

public class VisionSubsystem extends SubsystemBase {

  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputs[] m_inputs;
  private final Alert[] m_disconnectedAlerts;

  // Check for odometry initialization and use about //
  private int m_stablePoseCounter = 5;
  private boolean m_odometryInitialized = false;

  // Import VisionPoseValidator to validate our vision observations //
  private final VisionPoseValidator m_poseValidator = new VisionPoseValidator();

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
              "Vision camera " + io[i].getCameraName() + " is disconnected.", AlertType.kWarning);
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  private sealed interface VisionResult permits ValidPose, InvalidPose, NoTargets {}

  private record ValidPose(Pose2d pose, double confidence) implements VisionResult {}

  private record InvalidPose(String reason) implements VisionResult {}

  private record NoTargets() implements VisionResult {}

  @Override
  public void periodic() {
    // Use local variables to reduce field access (optimization)
    final var cameraIOs = m_io;
    final var cameraInputs = m_inputs;

    // This method will be called once per scheduler run
    for (int i = 0; i < cameraIOs.length; i++) {
      cameraIOs[i].updateInputs(cameraInputs[i]);
    }

    // Initialize logging values
    LinkedList<Pose3d> allTagPoses = new LinkedList<Pose3d>();
    LinkedList<Pose3d> allRobotPoses = new LinkedList<Pose3d>();
    LinkedList<Pose3d> allRobotPosesAccepted = new LinkedList<Pose3d>();
    LinkedList<Pose3d> allRobotPosesRejected = new LinkedList<Pose3d>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < cameraIOs.length; cameraIndex++) {
      var result = processCameraData(cameraIndex, cameraInputs[cameraIndex]);

      // Efficient collection operations
      allTagPoses.addAll(result.tagPoses());
      allRobotPoses.addAll(result.robotPoses());
      allRobotPosesAccepted.addAll(result.acceptedPoses());
      allRobotPosesRejected.addAll(result.rejectedPoses());
    }

    DogLog.log("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    DogLog.log("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
    DogLog.log("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
    DogLog.log("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
  }

  // Record for camera processing results
  private record CameraProcessingResult(
      LinkedList<Pose3d> tagPoses,
      LinkedList<Pose3d> robotPoses,
      LinkedList<Pose3d> acceptedPoses,
      LinkedList<Pose3d> rejectedPoses) {}

  private CameraProcessingResult processCameraData(int cameraIndex, VisionIOInputs inputs) {
    // Update disconnected alert
    m_disconnectedAlerts[cameraIndex].set(!inputs.connected);

    var tagPoses = new LinkedList<Pose3d>();
    var robotPoses = new LinkedList<Pose3d>();
    var acceptedPoses = new LinkedList<Pose3d>();
    var rejectedPoses = new LinkedList<Pose3d>();

    // Add tag poses
    for (int tagId : inputs.tagIds) {
      var tagPose = APTAG_FIELD_LAYOUT.getTagPose(tagId);
      if (tagPose.isPresent()) {
        tagPoses.add(tagPose.get());
      }
    }

    // Process pose observations with sealed classes
    for (var observation : inputs.poseObservations) {
      robotPoses.add(observation.pose());

      // Use instanceof with pattern matching
      var validationResult = m_poseValidator.validatePose(observation);

      if (validationResult instanceof AcceptedPose accepted) {
        acceptedPoses.add(accepted.poseObservation().pose());

        // Add to pose estimator
        var stdDevs = calculateStandardDeviations(accepted);
        m_consumer.accept(
            accepted.poseObservation().pose().toPose2d(),
            accepted.poseObservation().timestamp(),
            stdDevs);

        // Check if odometry is initialized
        if (!m_odometryInitialized) {
          m_stablePoseCounter--;
          if (m_stablePoseCounter <= 0) {
            m_odometryInitialized = true;
            // System.out.println("VisionSubsystem: Odometry initialized with vision.");
            DogLog.log("Vision/OdometryInitialized", true);
          }
        }

        // Log acceptance
        DogLog.log(
            "Vision/Camera" + cameraIndex + "/AcceptedPose", accepted.poseObservation().pose());
      } else if (validationResult instanceof RejectedPose rejected) {

        // If odometry not initialized, count down stable poses
        if (!m_odometryInitialized) {
          m_stablePoseCounter = 5; // Reset counter
        }

        rejectedPoses.add(rejected.poseObservation().pose());

        // Log rejection with reason
        System.out.printf(
            "Camera %d: Rejected pose - %s: %s%n",
            cameraIndex, rejected.reason().getDescription(), rejected.details());

        DogLog.log("Vision/Camera" + cameraIndex + "/RejectionReason", rejected.reason().name());
        DogLog.log(
            "Vision/Camera" + cameraIndex + "/RejectedPose", rejected.poseObservation().pose());
      }
    }

    return new CameraProcessingResult(tagPoses, robotPoses, acceptedPoses, rejectedPoses);
  }

  private Matrix<N3, N1> calculateStandardDeviations(AcceptedPose accepted) {
    // Your existing standard deviation calculation logic
    double stdDevFactor =
        (1 + accepted.poseObservation().averageTagDistance())
            * (1 + accepted.poseObservation().ambiguity())
            / Math.sqrt(Math.max(accepted.poseObservation().tagCount(), 1));
    double linearStdDev = LINEAR_STDDEV_BASELINE * stdDevFactor;
    double angularStdDev = ANGULAR_STDDEV_BASELINE * stdDevFactor;

    // --- Calculate Dynamic Per-Camera/Observation Factor ---
    // Start with a base factor of 1.0
    double dynamicCamFactor = 1.0;

    // Increase factor based on latency: Higher latency = less trustworthy
    // Example: Add 1.0 to the factor for every 100ms of latency. Adjust the
    // multiplier (10.0) as needed.
    dynamicCamFactor += accepted.poseObservation().latencySeconds() * 10.0;

    // Increase factor based on edge proximity: Higher edgeFactor = less trustworthy
    // Since edgeFactor is already >= 1.0, we can multiply by it directly,
    // or use a function of it if we want a different scaling.
    // Example: Multiply by edgeFactor. If edgeFactor is 1.5, uncertainty increases
    // by 50%.
    dynamicCamFactor *= accepted.poseObservation().edgeFactor();

    // Apply the calculated dynamic factor
    linearStdDev *= dynamicCamFactor;
    angularStdDev *= dynamicCamFactor;

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
