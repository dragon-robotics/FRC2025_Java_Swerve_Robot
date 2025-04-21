package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import static frc.robot.Constants.FieldConstants.*;

import java.util.function.Supplier;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private static VisionSystemSim m_visionSim;
  private final PhotonCameraSim m_cameraSim;

  private final Supplier<SwerveDriveState> m_swerveDriveStateSupplier;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<SwerveDriveState> swerveDriveStateSupplier) {
    super(name, robotToCamera, swerveDriveStateSupplier);
    m_swerveDriveStateSupplier = swerveDriveStateSupplier;

    // Initialize vision sim
    if (m_visionSim == null) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(APTAG_FIELD_LAYOUT);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    // cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    // cameraProperties.setCalibError(0.64, 0.25);
    // cameraProperties.setFPS(50);
    // cameraProperties.setAvgLatencyMs(25);
    // cameraProperties.setLatencyStdDevMs(10);
    
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties, APTAG_FIELD_LAYOUT);

    m_visionSim.addCamera(m_cameraSim, robotToCamera);

    m_cameraSim.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSim.update(m_swerveDriveStateSupplier.get().Pose);
    super.updateInputs(inputs);
  }

}
