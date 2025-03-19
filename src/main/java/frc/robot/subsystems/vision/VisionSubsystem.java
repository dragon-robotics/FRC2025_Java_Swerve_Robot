// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

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

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera aprilTagAlignLeftCamera;
  private final PhotonCamera aprilTagAlignRightCamera;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private final PhotonPoseEstimator photonEstimatorRight;
  private Matrix<N3, N1> curStdDevs;

  // Robot state
  private SwerveDriveState driveState;

  // Simulation
  private PhotonCameraSim aprilTagAlignLeftCameraSim;
  private PhotonCameraSim aprilTagAlignRightCameraSim;
  private VisionSystemSim visionSim;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveDriveState driveState) {
    aprilTagAlignLeftCamera = new PhotonCamera(APTAG_CAMERA_NAMES[0]);
    aprilTagAlignRightCamera = new PhotonCamera(APTAG_CAMERA_NAMES[1]);

    photonEstimatorLeft = new PhotonPoseEstimator(
        APTAG_FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        APTAG_ALIGN_LEFT_CAM_POS);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    photonEstimatorRight = new PhotonPoseEstimator(
        APTAG_FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        APTAG_ALIGN_RIGHT_CAM_POS);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    this.driveState = driveState;

    // ----- Simulation
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the
      // field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this
      // simulated field.
      visionSim.addAprilTags(APTAG_FIELD_LAYOUT);
      // Create simulated camera properties. These can be set to mimic your actual
      // camera.
      var cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(72));
      cameraProp.setCalibError(0.64, 0.10);
      cameraProp.setFPS(50);
      cameraProp.setAvgLatencyMs(25);
      cameraProp.setLatencyStdDevMs(5);
      
      // Create a PhotonCameraSim which will update the linked PhotonCamera's values
      // with visible targets.
      aprilTagAlignLeftCameraSim = new PhotonCameraSim(aprilTagAlignLeftCamera, cameraProp);
      aprilTagAlignRightCameraSim = new PhotonCameraSim(aprilTagAlignRightCamera, cameraProp);
      
      // Add the simulated camera to view the targets on this simulated field.
      visionSim.addCamera(aprilTagAlignLeftCameraSim, APTAG_ALIGN_LEFT_CAM_POS);
      visionSim.addCamera(aprilTagAlignRightCameraSim, APTAG_ALIGN_RIGHT_CAM_POS);

      aprilTagAlignLeftCameraSim.enableDrawWireframe(true);
      aprilTagAlignRightCameraSim.enableDrawWireframe(true);
    }
  }

  /**
   * Gets a fused pose estimate from multiple cameras.
   * @return An optional containing the fused robot pose if available.
   */
  public Optional<EstimatedRobotPose> getFusedRobotPose() {
    Optional<EstimatedRobotPose> leftPose = photonEstimatorLeft.update(aprilTagAlignLeftCamera.getLatestResult());
    Optional<EstimatedRobotPose> rightPose = photonEstimatorRight.update(aprilTagAlignRightCamera.getLatestResult());
    
    // 1. Strategy: Choose the "best" estimate based on quality metrics
    if (leftPose.isPresent() && rightPose.isPresent()) {
        // Determine quality factors for each pose estimate
        int leftTagCount = leftPose.get().targetsUsed.size();
        int rightTagCount = rightPose.get().targetsUsed.size(); 
        
        // Get the average ambiguity for each camera
        double leftAmbiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
        double rightAmbiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
        
        // Choose the better estimate: prefer more tags, then lower ambiguity
        if (leftTagCount > rightTagCount) {
            curStdDevs = calculateStdDevs(leftTagCount, leftAmbiguity);
            return leftPose;
        } else if (rightTagCount > leftTagCount) {
            curStdDevs = calculateStdDevs(rightTagCount, rightAmbiguity);
            return rightPose;
        } else {
            // Same number of tags, choose based on ambiguity
            if (leftAmbiguity <= rightAmbiguity) {
                curStdDevs = calculateStdDevs(leftTagCount, leftAmbiguity);
                return leftPose;
            } else {
                curStdDevs = calculateStdDevs(rightTagCount, rightAmbiguity);
                return rightPose;
            }
        }
    } else if (leftPose.isPresent()) {
        int tagCount = leftPose.get().targetsUsed.size();
        double ambiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
        curStdDevs = calculateStdDevs(tagCount, ambiguity);
        return leftPose;
    } else if (rightPose.isPresent()) {
        int tagCount = rightPose.get().targetsUsed.size();
        double ambiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
        curStdDevs = calculateStdDevs(tagCount, ambiguity);
        return rightPose;
    }
    
    return Optional.empty();
  }

/**
 * Advanced fusion using weighted averaging of poses.
 */
public Optional<EstimatedRobotPose> getFusedRobotPoseWeighted() {
    Optional<EstimatedRobotPose> leftPose = photonEstimatorLeft.update(aprilTagAlignLeftCamera.getLatestResult());
    Optional<EstimatedRobotPose> rightPose = photonEstimatorRight.update(aprilTagAlignRightCamera.getLatestResult());
    
    if (!leftPose.isPresent() && !rightPose.isPresent()) {
        return Optional.empty();
    } else if (!leftPose.isPresent()) {
        return rightPose;
    } else if (!rightPose.isPresent()) {
        return leftPose;
    }
    
    // Both poses exist - calculate quality weights
    Pose2d leftPose2d = leftPose.get().estimatedPose.toPose2d();
    Pose2d rightPose2d = rightPose.get().estimatedPose.toPose2d();
    
    int leftTagCount = leftPose.get().targetsUsed.size();
    int rightTagCount = rightPose.get().targetsUsed.size();
    double leftAmbiguity = getAverageAmbiguity(aprilTagAlignLeftCamera.getLatestResult());
    double rightAmbiguity = getAverageAmbiguity(aprilTagAlignRightCamera.getLatestResult());
    
    // Calculate weights based on tag count and ambiguity
    double leftWeight = leftTagCount / (1.0 + leftAmbiguity);
    double rightWeight = rightTagCount / (1.0 + rightAmbiguity);
    double totalWeight = leftWeight + rightWeight;
    
    // Normalized weights
    leftWeight /= totalWeight;
    rightWeight /= totalWeight;
    
    // Weighted average of positions
    double x = leftPose2d.getX() * leftWeight + rightPose2d.getX() * rightWeight;
    double y = leftPose2d.getY() * leftWeight + rightPose2d.getY() * rightWeight;
    
    // For rotation, interpolate between the two angles
    Rotation2d rot = leftPose2d.getRotation().interpolate(rightPose2d.getRotation(), rightWeight);
    
    // Create fused pose
    Pose2d fusedPose = new Pose2d(x, y, rot);
    
    // Calculate standard deviations based on quality of estimates
    curStdDevs = calculateStdDevs(leftTagCount + rightTagCount, 
                                 (leftAmbiguity * leftWeight + rightAmbiguity * rightWeight));
    
    // Create a new estimated robot pose with the newest timestamp
    return Optional.of(new EstimatedRobotPose(
        new edu.wpi.first.math.geometry.Pose3d(fusedPose), 
        Math.max(leftPose.get().timestampSeconds, rightPose.get().timestampSeconds),
        leftPose.get().targetsUsed,  // Use one camera's targets or combine them if needed
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    ));
  }  

  /**
   * Calculates average ambiguity from a result with targets.
   */
  private double getAverageAmbiguity(PhotonPipelineResult result) {
      if (!result.hasTargets()) {
          return Double.MAX_VALUE;
      }
      
      return result.getTargets().stream()
          .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
          .average()
          .orElse(Double.MAX_VALUE);
  }

  /**
   * Calculate standard deviations based on the quality of pose estimation.
   */
  private Matrix<N3, N1> calculateStdDevs(int tagCount, double ambiguity) {
    // Base standard deviations - adjust these based on your robot/camera setup
    double baseXY = 0.5; // meters
    double baseTheta = 0.5; // radians
    
    // Lower the std devs with more tags and lower ambiguity
    double xyStdDev = baseXY / Math.sqrt(tagCount) * (1 + ambiguity);
    double thetaStdDev = baseTheta / Math.sqrt(tagCount) * (1 + ambiguity);
    
    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

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

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation())
      visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation())
      return null;
    return visionSim.getDebugField();
  }

  public PhotonCamera getCamera(boolean isLeft) {
    return isLeft ? aprilTagAlignLeftCamera : aprilTagAlignRightCamera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      visionSim.update(driveState.Pose);
    }
  }
}
