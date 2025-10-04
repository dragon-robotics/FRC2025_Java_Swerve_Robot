package frc.robot.subsystems.vision;

import static frc.robot.Constants.FieldConstants.APTAG_FIELD_LAYOUT;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator poseEstimator;

  protected final Supplier<SwerveDriveState> swerveDriveStateSupplier;

  /**
   * Creates a new VisionIOPV.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param rotationSupplier The current heading of the robot.
   */
  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<SwerveDriveState> swerveDriveStateSupplier) {
    this.camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.swerveDriveStateSupplier = swerveDriveStateSupplier;
    poseEstimator =
        new PhotonPoseEstimator(
            APTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

    // Reset heading data before pose initialization //
    poseEstimator.resetHeadingData(
        Timer.getFPGATimestamp(), swerveDriveStateSupplier.get().Pose.getRotation());
  }

  @Override
  public String getCameraName() {
    // Get the camera object
    return camera.getName();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.setConnected(camera.isConnected());
    inputs.setCameraName(camera.getName());

    // Update pose estimation heading data //
    poseEstimator.addHeadingData(
        Timer.getFPGATimestamp(), swerveDriveStateSupplier.get().Pose.getRotation());

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      inputs.setLatestTargetObservation(new TargetObservation(new Rotation2d(), new Rotation2d()));
      if (result.hasTargets()) {
        inputs.setLatestTargetObservation(
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch())));
      }

      // Add pose observation based on how many tags we see
      Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);

      visionEst.ifPresent(
          estimator -> {
            // Get robot pose
            Pose3d robotPose = estimator.estimatedPose;

            // Calculate average tag distance
            double averageTagDistance =
                result.getTargets().stream()
                    .mapToDouble(t -> t.bestCameraToTarget.getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

            // Add tag IDs
            List<Short> tagIdsUsed =
                result.getTargets().stream().map(target -> (short) target.getFiducialId()).toList();
            tagIds.addAll(tagIdsUsed);

            // Calculate average ambiguity across all visible tags
            double ambiguity =
                result.getTargets().stream()
                    .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                    .average()
                    .orElse(0.0);

            // Calculate latency
            double latencySeconds = result.metadata.getLatencyMillis() / 1000;

            // Calculate Edge Factor //
            double yawThreshold =
                VisionConstants.CAMERA_FOV_HORIZONTAL_DEGREES / 2.0 * 0.8; // e.g., 80% of half-FOV
            double pitchThreshold = VisionConstants.CAMERA_FOV_VERTICAL_DEGREES / 2.0 * 0.8;
            double edgeFactor =
                result.getTargets().stream()
                    .mapToDouble(
                        target -> {
                          double targetYaw = Math.abs(target.getYaw());
                          double targetPitch = Math.abs(target.getPitch());
                          double yawScore =
                              Math.max(0, targetYaw - yawThreshold * 0.5) / (yawThreshold * 0.5);
                          double pitchScore =
                              Math.max(0, targetPitch - pitchThreshold * 0.5)
                                  / (pitchThreshold * 0.5);
                          return 1.0 + Math.max(yawScore, pitchScore);
                        })
                    .max()
                    .orElse(1.0);

            // Add pose observation calculated from the PhotonPoseEstimator
            poseObservations.add(
                new PoseObservation(
                    estimator.timestampSeconds, // Timestamp
                    robotPose, // 3D pose estimate
                    ambiguity, // Ambiguity
                    result.getTargets().size(), // Tag count
                    averageTagDistance, // Average tag distance
                    latencySeconds, // Latency
                    edgeFactor, // Edge factor
                    PoseObservationType.PHOTONVISION)); // Observation type
          });
    }

    // Save pose observations to inputs object
    inputs.setPoseObservations(poseObservations.toArray(new PoseObservation[0]));

    // Save tag IDs to inputs objects
    inputs.setTagIds(tagIds.stream().mapToInt(Short::shortValue).toArray());
  }
}
