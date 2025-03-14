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
  private final PhotonPoseEstimator photonEstimator;
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

    photonEstimator = new PhotonPoseEstimator(APTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        APTAG_ALIGN_LEFT_CAM_POS);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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
      cameraProp.setCalibError(0.35, 0.10);
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
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   *
   * <p>
   * Also includes updates for the standard deviations, which can (optionally) be
   * retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : aprilTagAlignLeftCamera.getAllUnreadResults()) {
      visionEst = photonEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());

      if (Robot.isSimulation()) {
        visionEst.ifPresentOrElse(
            est -> getSimDebugField()
                .getObject("VisionEstimation")
                .setPose(est.estimatedPose.toPose2d()),
            () -> {
              getSimDebugField().getObject("VisionEstimation").setPoses();
            });
      }
    }
    return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = SINGLE_TAG_STDDEV;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = SINGLE_TAG_STDDEV;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = SINGLE_TAG_STDDEV;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = MULTI_TAG_STDDEV;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

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
