package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.APTAG_FIELD_LAYOUT;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOPhotonVision implements VisionIO {
	protected final PhotonCamera m_camera;
	protected final Transform3d m_robotToCamera;
  protected final PhotonPoseEstimator m_poseEstimator;

  protected final Supplier<SwerveDriveState> m_swerveDriveStateSupplier;

  /**
   * Creates a new VisionIOPV.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param rotationSupplier The current heading of the robot.
   */
  public VisionIOPhotonVision(
      String name,
      Transform3d robotToCamera,
      Supplier<SwerveDriveState> swerveDriveStateSupplier) {
    m_camera = new PhotonCamera(name);
    m_robotToCamera = robotToCamera;
    m_swerveDriveStateSupplier = swerveDriveStateSupplier;
    m_poseEstimator =
        new PhotonPoseEstimator(
            APTAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            m_robotToCamera);
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = m_camera.isConnected();

    // Update pose estimation heading data //
    m_poseEstimator.addHeadingData(Timer.getFPGATimestamp(), m_swerveDriveStateSupplier.get().Pose.getRotation());

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : m_camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // Add pose observation based on how many tags we see
      visionEst = m_poseEstimator.update(result);

      visionEst.ifPresent(
          estimator -> {
            // Get robot pose
            Pose3d robotPose = estimator.estimatedPose;

            // Calculate tag distance
            double totalTagDistance = 0.0;
            for (var target : result.targets) {
              totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
            }

            // Calculate average tag distance if multiple tags are used
            if (result.getTargets().size() > 0) {
              totalTagDistance /= result.getTargets().size();
            }

            // Add tag IDs
            List<Short> tagIdsUsed = result.getTargets().stream()
                .map(target -> (short) target.getFiducialId())
                .collect(Collectors.toList());
            tagIds.addAll(tagIdsUsed);

            // Calculate average ambiguity across all visible tags
            double ambiguity =
              result.getTargets().stream()
                .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                .average()
                .orElse(0.0);

            // Add pose observation calculated from the PhotonPoseEstimator
            poseObservations.add(
                new PoseObservation(
                    estimator.timestampSeconds, // Timestamp
                    robotPose, // 3D pose estimate
                    ambiguity, // Ambiguity
                    result.getTargets().size(), // Tag count
                    totalTagDistance, // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
      );
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }  
}
