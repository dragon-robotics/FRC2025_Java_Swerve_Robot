package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {

  public static class VisionIOInputs {
    public String cameraName = "";
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp, // seconds since of the pose observation
      Pose3d pose,  // pose of the robot in the camera frame
      double ambiguity, // ambiguity of the pose estimate
      int tagCount, // number of tags used to estimate the pose
      double averageTagDistance, // average distance to the tags used to estimate the pose
      double latencySeconds, // latency of the pose estimate in seconds
      double edgeFactor, // how close the tags are to the edge of the camera frame
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getCameraName() {return "";}
}
