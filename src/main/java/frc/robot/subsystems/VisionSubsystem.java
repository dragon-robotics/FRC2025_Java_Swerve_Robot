// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera apTagAlignCamera;

  private final PhotonCamera[] poseCameras;
  private final PhotonPoseEstimator[] photonPoseEstimators;
  
  // List of standard deviations for the pose estimation for each pose estimation camera
  private ArrayList<Matrix<N3, N1>> curPoseStdDevs;

  // Simulation
  private PhotonCameraSim[] cameraSims;
  private VisionSystemSim visionSim;

  // Robot state
  private SwerveDriveState driveState;

  StructPublisher<Pose3d> publisher =
      NetworkTableInstance
          .getDefault()
          .getStructTopic("estimatedPose", Pose3d.struct)
          .publish();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveDriveState driveState) {

    apTagAlignCamera = new PhotonCamera(APTAG_CAMERA_NAMES[0]);

    curPoseStdDevs = new ArrayList<Matrix<N3, N1>>(APTAG_POSE_EST_CAM_POSITIONS.length);

    poseCameras = new PhotonCamera[APTAG_POSE_EST_CAM_POSITIONS.length];
    photonPoseEstimators = new PhotonPoseEstimator[APTAG_POSE_EST_CAM_POSITIONS.length];

    for (int i = 1; i <= APTAG_POSE_EST_CAM_POSITIONS.length; i++) {
      poseCameras[i - 1] = new PhotonCamera(APTAG_CAMERA_NAMES[i]);
      photonPoseEstimators[i - 1] = new PhotonPoseEstimator(
          APTAG_FIELD_LAYOUT,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          APTAG_POSE_EST_CAM_POSITIONS[i]
      );
      photonPoseEstimators[i - 1].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    this.driveState = driveState;

    // ----- Simulation
    if (Robot.isSimulation()) {
        
      // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(APTAG_FIELD_LAYOUT);
        
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(90);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(5);
        
        cameraSims = new PhotonCameraSim[poseCameras.length];
        for (int i = 0; i < poseCameras.length; i++) {
          cameraSims[i] = new PhotonCameraSim(poseCameras[i], cameraProp);
          visionSim.addCamera(cameraSims[i], APTAG_POSE_EST_CAM_POSITIONS[i]);
          cameraSims[i].enableDrawWireframe(true);
        }
    }
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (int i = 0; i < poseCameras.length; i++) {
          var camera = poseCameras[i];
          var photonEstimator = photonPoseEstimators[i];
          for (var change : camera.getAllUnreadResults()) {
              visionEst = photonEstimator.update(change);
              updateEstimationStdDevs(visionEst, change.getTargets(), i);

              if (Robot.isSimulation()) {
                  visionEst.ifPresentOrElse(est -> {
                      publisher.set(est.estimatedPose);
                  }, () -> {
                      publisher.set(new Pose3d());
                  });
              }
          }
      }
      return visionEst;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
          Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, int cameraIndex) {
      if (estimatedPose.isEmpty()) {
          // No pose input. Default to single-tag std devs
          curPoseStdDevs.set(cameraIndex, SINGLE_TAG_STDDEV);

      } else {
          // Pose present. Start running Heuristic
          var estStdDevs = SINGLE_TAG_STDDEV;
          int numTags = 0;
          double avgDist = 0;

          // Precalculation - see how many tags we found, and calculate an average-distance metric
          for (var tgt : targets) {
              var tagPose = photonPoseEstimators[cameraIndex].getFieldTags().getTagPose(tgt.getFiducialId());
              if (tagPose.isEmpty()) continue;
              numTags++;
              avgDist +=
                      tagPose
                              .get()
                              .toPose2d()
                              .getTranslation()
                              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
          }

          if (numTags == 0) {
              // No tags visible. Default to single-tag std devs
              curPoseStdDevs.set(cameraIndex, SINGLE_TAG_STDDEV);
          } else {
              // One or more tags visible, run the full heuristic.
              avgDist /= numTags;
              // Decrease std devs if multiple targets are visible
              if (numTags > 1) estStdDevs = MULTI_TAG_STDDEV;
              // Increase std devs based on (average) distance
              if (numTags == 1 && avgDist > 4)
                  estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
              else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
              curPoseStdDevs.set(cameraIndex, estStdDevs);
          }
      }
  }

  public PhotonCamera getApTagAlignCamera() {
    return apTagAlignCamera;
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs(int cameraIndex) {
      return curPoseStdDevs.get(cameraIndex);
  }

  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return visionSim.getDebugField();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      visionSim.update(driveState.Pose);
      // getEstimatedGlobalPose();
    }
  }
}
