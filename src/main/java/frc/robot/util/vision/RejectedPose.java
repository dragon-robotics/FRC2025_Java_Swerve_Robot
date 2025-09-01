package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

// Record for rejected poses with reason
public record RejectedPose(
    PoseObservation poseObservation,
    RejectionReason reason,
    String details
) implements PoseValidationResult {}
