package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meter;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPVSim implements VisionIO {

  private PhotonCamera aprilTagAlignLeftCamera;
  private PhotonCamera aprilTagAlignRightCamera;

  // Simulation
  private PhotonCameraSim aprilTagAlignLeftCameraSim;
  private PhotonCameraSim aprilTagAlignRightCameraSim;
  private VisionSystemSim visionSim;

  // Target ID to track
  private int targetId;

  public VisionIOPVSim() {

    // Create the vision system simulation which handles cameras and targets on the
    // field.
    visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(APTAG_FIELD_LAYOUT);

    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(30);
    cameraProp.setAvgLatencyMs(40);
    cameraProp.setLatencyStdDevMs(15);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible
    // targets.
    aprilTagAlignLeftCameraSim = new PhotonCameraSim(aprilTagAlignLeftCamera, cameraProp);
    aprilTagAlignRightCameraSim = new PhotonCameraSim(aprilTagAlignRightCamera, cameraProp);

    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(aprilTagAlignLeftCameraSim, APTAG_ALIGN_LEFT_CAM_POS);
    visionSim.addCamera(aprilTagAlignLeftCameraSim, APTAG_ALIGN_RIGHT_CAM_POS);

    // Enable drawing wireframe for both cameras in simulation
    aprilTagAlignLeftCameraSim.enableDrawWireframe(true);
    aprilTagAlignRightCameraSim.enableDrawWireframe(true);
  }

  @Override
  public void setTargetId(int id) {
    targetId = id;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    List<PhotonPipelineResult> leftCameraResults =
        aprilTagAlignLeftCameraSim.getCamera().getAllUnreadResults();

    // Check if left camera has any unread results
    inputs.hasTargets[0] = !leftCameraResults.isEmpty();

    // Get the last result timestamp for the left camera if there are any unread results
    if (inputs.hasTargets[0]) {
      // Get the last result for the left camera //
      PhotonPipelineResult leftCamLastResult =
          leftCameraResults.get(leftCameraResults.size() - 1); // This is the last result

      // Get the last timestamp for the left camera //
      inputs.lastTimestamp[0] = leftCamLastResult.getTimestampSeconds();

      // Get the best tag id for the left camera //
      inputs.tagId[0] = leftCamLastResult.getBestTarget().getFiducialId();

      // Get the yaw and range for the best tag id for the left camera //
      inputs.targetYawAndRange[0][0] = leftCamLastResult.getBestTarget().getYaw();
      inputs.targetYawAndRange[0][1] =
          PhotonUtils.calculateDistanceToTargetMeters(
              APTAG_ALIGN_LEFT_CAM_POS.getMeasureZ().in(Meter),
              Units.inchesToMeters(12.125),
              APTAG_ALIGN_LEFT_CAM_POS.getRotation().getY(),
              leftCamLastResult.getBestTarget().getPitch());
    }

    List<PhotonPipelineResult> rightCameraResults =
        aprilTagAlignRightCameraSim.getCamera().getAllUnreadResults();

    // Check if right camera has any unread results
    inputs.hasTargets[1] = !rightCameraResults.isEmpty();

    // Get the last result timestamp for the right camera if there are any unread results
    if (inputs.hasTargets[1]) {
      // Get the last result for the right camera //
      PhotonPipelineResult rightCamLastResult =
          rightCameraResults.get(rightCameraResults.size() - 1); // This is the last result

      // Get the last timestamp for the right camera //
      inputs.lastTimestamp[1] = rightCamLastResult.getTimestampSeconds();

      // Get the best tag id for the right camera //
      inputs.tagId[1] = rightCamLastResult.getBestTarget().getFiducialId();

      // Get the yaw and range for the best tag id for the right camera //
      inputs.targetYawAndRange[1][0] = rightCamLastResult.getBestTarget().getYaw();
      inputs.targetYawAndRange[1][1] =
          PhotonUtils.calculateDistanceToTargetMeters(
              APTAG_ALIGN_RIGHT_CAM_POS.getMeasureZ().in(Meter),
              Units.inchesToMeters(12.125),
              APTAG_ALIGN_RIGHT_CAM_POS.getRotation().getY(),
              rightCamLastResult.getBestTarget().getPitch());
    }

    inputs.targetTagId = targetId;
  }
}
