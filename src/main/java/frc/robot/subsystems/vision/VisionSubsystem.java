// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {

  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputs[] m_inputs;
  private final Alert[] m_disconnectedAlerts;

  StructArrayPublisher<Pose3d> m_tagPosesArrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision/Summary/TagPoses", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> m_robotPosesArrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision/Summary/RobotPoses", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> m_estRobotPosesArrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision/Summary/EstimateRobotPoses", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> m_robotPosesAcceptedArrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision/Summary/RobotPosesAccepted", Pose3d.struct).publish();
  StructArrayPublisher<Pose3d> m_robotPosesRejectedArrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Vision/Summary/RobotPosesRejected", Pose3d.struct).publish();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
    m_consumer = consumer;
    m_io = io;

    // Initialize the the inputs
    m_inputs = new VisionIOInputs[m_io.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputs();
    }

    // Initialize the disconnected alerts
    // Initialize disconnected alerts
    m_disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  // /**
  //  * Gets a fused pose estimate from multiple cameras.
  //  * @return An optional containing the fused robot pose if available.
  //  */
  // public Optional<EstimatedRobotPose> getFusedRobotPose() {
  //   Optional<EstimatedRobotPose> leftPose = photonEstimatorLeft.update(aprilTagAlignLeftCamera.getLatestResult());
  //   Optional<EstimatedRobotPose> rightPose = photonEstimatorRight.update(aprilTagAlignRightCamera.getLatestResult());
    
  //   // 1. Strategy: Choose the "best" estimate based on quality metrics
  //   if (leftPose.isPresent() && rightPose.isPresent()) {
  //       // Determine quality factors for each pose estimate
  //       int leftTagCount = leftPose.get().targetsUsed.size();
  //       int rightTagCount = rightPose.get().targetsUsed.size(); 
        
  //       // Get the average ambiguity for each camera
  //       double leftAmbiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
  //       double rightAmbiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
        
  //       // Choose the better estimate: prefer more tags, then lower ambiguity
  //       if (leftTagCount > rightTagCount) {
  //           curStdDevs = calculateStdDevs(leftTagCount, leftAmbiguity);
  //           return leftPose;
  //       } else if (rightTagCount > leftTagCount) {
  //           curStdDevs = calculateStdDevs(rightTagCount, rightAmbiguity);
  //           return rightPose;
  //       } else {
  //           // Same number of tags, choose based on ambiguity
  //           if (leftAmbiguity <= rightAmbiguity) {
  //               curStdDevs = calculateStdDevs(leftTagCount, leftAmbiguity);
  //               return leftPose;
  //           } else {
  //               curStdDevs = calculateStdDevs(rightTagCount, rightAmbiguity);
  //               return rightPose;
  //           }
  //       }
  //   } else if (leftPose.isPresent()) {
  //       int tagCount = leftPose.get().targetsUsed.size();
  //       double ambiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
  //       curStdDevs = calculateStdDevs(tagCount, ambiguity);
  //       return leftPose;
  //   } else if (rightPose.isPresent()) {
  //       int tagCount = rightPose.get().targetsUsed.size();
  //       double ambiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
  //       curStdDevs = calculateStdDevs(tagCount, ambiguity);
  //       return rightPose;
  //   }
    
  //   return Optional.empty();
  // }

// /**
//  * Advanced fusion using weighted averaging of poses.
//  */
// public Optional<EstimatedRobotPose> getFusedRobotPoseWeighted() {
//     Optional<EstimatedRobotPose> leftPose = photonEstimatorLeft.update(aprilTagAlignLeftCamera.getLatestResult());
//     Optional<EstimatedRobotPose> rightPose = photonEstimatorRight.update(aprilTagAlignRightCamera.getLatestResult());
    
//     if (!leftPose.isPresent() && !rightPose.isPresent()) {
//         return Optional.empty();
//     } else if (!leftPose.isPresent()) {
//         return rightPose;
//     } else if (!rightPose.isPresent()) {
//         return leftPose;
//     }
    
//     // Both poses exist - calculate quality weights
//     Pose2d leftPose2d = leftPose.get().estimatedPose.toPose2d();
//     Pose2d rightPose2d = rightPose.get().estimatedPose.toPose2d();
    
//     int leftTagCount = leftPose.get().targetsUsed.size();
//     int rightTagCount = rightPose.get().targetsUsed.size();
//     double leftAmbiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
//     double rightAmbiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
    
//     // Calculate weights based on tag count and ambiguity
//     double leftWeight = leftTagCount / (1.0 + leftAmbiguity);
//     double rightWeight = rightTagCount / (1.0 + rightAmbiguity);
//     double totalWeight = leftWeight + rightWeight;
    
//     // Normalized weights
//     leftWeight /= totalWeight;
//     rightWeight /= totalWeight;
    
//     // Weighted average of positions
//     double x = leftPose2d.getX() * leftWeight + rightPose2d.getX() * rightWeight;
//     double y = leftPose2d.getY() * leftWeight + rightPose2d.getY() * rightWeight;
    
//     // For rotation, interpolate between the two angles
//     Rotation2d rot = leftPose2d.getRotation().interpolate(rightPose2d.getRotation(), rightWeight);
    
//     // Create fused pose
//     Pose2d fusedPose = new Pose2d(x, y, rot);
    
//     // Calculate standard deviations based on quality of estimates
//     curStdDevs = calculateStdDevs(leftTagCount + rightTagCount, 
//                                  (leftAmbiguity * leftWeight + rightAmbiguity * rightWeight));
    
//     // Create a new estimated robot pose with the newest timestamp
//     return Optional.of(new EstimatedRobotPose(
//         new edu.wpi.first.math.geometry.Pose3d(fusedPose), 
//         Math.max(leftPose.get().timestampSeconds, rightPose.get().timestampSeconds),
//         leftPose.get().targetsUsed,  // Use one camera's targets or combine them if needed
//         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
//     ));
//   }  

//   /**
//    * Calculates average ambiguity from a result with targets.
//    */
//   private double getAverageAmbiguity(PhotonPipelineResult result) {
//       if (!result.hasTargets()) {
//           return Double.MAX_VALUE;
//       }
      
//       return result.getTargets().stream()
//           .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
//           .average()
//           .orElse(Double.MAX_VALUE);
//   }

//   /**
//    * Calculate standard deviations based on the quality of pose estimation.
//    */
//   private Matrix<N3, N1> calculateStdDevs(int tagCount, double ambiguity) {
//     // Base standard deviations - adjust these based on your robot/camera setup
//     double baseXY = 0.5; // meters
//     double baseTheta = 0.5; // radians
    
//     // Lower the std devs with more tags and lower ambiguity
//     double xyStdDev = baseXY / Math.sqrt(tagCount) * (1 + ambiguity);
//     double thetaStdDev = baseTheta / Math.sqrt(tagCount) * (1 + ambiguity);
    
//     return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
//   }

  // /**
  //  * The latest estimated robot pose on the field from vision data. This may be
  //  * empty. This should
  //  * only be called once per loop.
  //  *
  //  * <p>
  //  * Also includes updates for the standard deviations, which can (optionally) be
  //  * retrieved with
  //  * {@link getEstimationStdDevs}
  //  *
  //  * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
  //  *         timestamp, and targets
  //  *         used for estimation.
  //  */
  //   public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //     Optional<EstimatedRobotPose> visionEst = Optional.empty();
  //     for (var change : aprilTagAlignLeftCamera.getAllUnreadResults()) {
  //       visionEst = photonEstimator.update(change);
  //       updateEstimationStdDevs(visionEst, change.getTargets());

  //       if (Robot.isSimulation()) {
  //         visionEst.ifPresentOrElse(
  //             est -> getSimDebugField()
  //                 .getObject("VisionEstimation")
  //                 .setPose(est.estimatedPose.toPose2d()),
  //             () -> {
  //               getSimDebugField().getObject("VisionEstimation").setPoses();
  //             });
  //       }
  //     }
  //     return visionEst;
  //   }

  // /**
  //  * Calculates new standard deviations This algorithm is a heuristic that creates
  //  * dynamic standard
  //  * deviations based on number of tags, estimation strategy, and distance from
  //  * the tags.
  //  *
  //  * @param estimatedPose The estimated pose to guess standard deviations for.
  //  * @param targets       All targets in this camera frame
  //  */
  // private void updateEstimationStdDevs(
  //     Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
  //   if (estimatedPose.isEmpty()) {
  //     // No pose input. Default to single-tag std devs
  //     curStdDevs = SINGLE_TAG_STDDEV;

  //   } else {
  //     // Pose present. Start running Heuristic
  //     var estStdDevs = SINGLE_TAG_STDDEV;
  //     int numTags = 0;
  //     double avgDist = 0;

  //     // Precalculation - see how many tags we found, and calculate an
  //     // average-distance metric
  //     for (var tgt : targets) {
  //       var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
  //       if (tagPose.isEmpty())
  //         continue;
  //       numTags++;
  //       avgDist += tagPose
  //           .get()
  //           .toPose2d()
  //           .getTranslation()
  //           .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
  //     }

  //     if (numTags == 0) {
  //       // No tags visible. Default to single-tag std devs
  //       curStdDevs = SINGLE_TAG_STDDEV;
  //     } else {
  //       // One or more tags visible, run the full heuristic.
  //       avgDist /= numTags;
  //       // Decrease std devs if multiple targets are visible
  //       if (numTags > 1)
  //         estStdDevs = MULTI_TAG_STDDEV;
  //       // Increase std devs based on (average) distance
  //       if (numTags == 1 && avgDist > 4)
  //         estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  //       else
  //         estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
  //       curStdDevs = estStdDevs;
  //     }
  //   }
  // }

  // /**
  //  * Returns the latest standard deviations of the estimated pose from {@link
  //  * #getEstimatedGlobalPose()}, for use with {@link
  //  * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
  //  * SwerveDrivePoseEstimator}. This should
  //  * only be used when there are targets visible.
  //  */
  // public Matrix<N3, N1> getEstimationStdDevs() {
  //   return curStdDevs;
  // }

  // // ----- Simulation
  // public void simulationPeriodic(Pose2d robotSimPose) {
  //   visionSim.update(robotSimPose);
  // }

  // /** Reset pose history of the robot in the vision system simulation. */
  // public void resetSimPose(Pose2d pose) {
  //   if (Robot.isSimulation())
  //     visionSim.resetRobotPose(pose);
  // }

  // /** A Field2d for visualizing our robot and objects on the field. */
  // public Field2d getSimDebugField() {
  //   if (!Robot.isSimulation())
  //     return null;
  //   return visionSim.getDebugField();
  // }

  // public PhotonCamera getCamera(boolean isLeft) {
  //   return isLeft ? aprilTagAlignLeftCamera : aprilTagAlignRightCamera;
  // }

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
      // SignalLogger.
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPoseEstimates = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < m_io.length; cameraIndex++) {
      // Update disconnected alert
      m_disconnectedAlerts[cameraIndex].set(!m_inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPoseEstimates = new LinkedList<>();
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
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
      //     tagPoses.toArray(new Pose3d[tagPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
      //     robotPoses.toArray(new Pose3d[robotPoses.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
      //     robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      // Logger.recordOutput(
      //     "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
      //     robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    m_tagPosesArrayPublisher.set(allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesArrayPublisher.set(allRobotPoses.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesArrayPublisher.set(allRobotPoses.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesAcceptedArrayPublisher.set(allRobotPosesAccepted.toArray(new Pose3d[allTagPoses.size()]));
    m_robotPosesRejectedArrayPublisher.set(allRobotPosesRejected.toArray(new Pose3d[allTagPoses.size()]));
  }
}
