// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // public static final Mode currentMode = Mode.REAL;

  // public static enum Mode {
  // /** Running on a real robot. */
  // REAL,

  // /** Running a physics simulator. */
  // SIM,

  // /** Replaying from a log file. */
  // REPLAY
  // }

  /** General robot constants */
  public static final class GeneralConstants {

    // Robot mode
    public static final RobotMode CURRENT_MODE = RobotBase.isReal() ? RobotMode.COMP : RobotMode.SIM;

    public static enum RobotMode {
      /** Running on test mode */
      TEST,
      /** Running on competition mode */
      COMP,
      /** Running on simulation mode */
      SIM
    }

    public static boolean disableHAL = false;
  }

  public static class FieldConstants {
    // Robot heading constants for different field elements //
    public static final Rotation2d LEFT_CORAL_STATION_INTAKE_ANGLE = Rotation2d.fromDegrees(-54.011);
    public static final Rotation2d RIGHT_CORAL_STATION_INTAKE_ANGLE = Rotation2d.fromDegrees(54.011);
    public static final Rotation2d ALGAE_PROCESSOR_STATION_ANGLE = Rotation2d.fromDegrees(-90);

    // Blue Reef Station ID Angle Constant //
    public static final Rotation2d REEF_STATION_ID_17_ANGLE = Rotation2d.fromDegrees(60);
    public static final Rotation2d REEF_STATION_ID_18_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d REEF_STATION_ID_19_ANGLE = Rotation2d.fromDegrees(-60);
    public static final Rotation2d REEF_STATION_ID_20_ANGLE = Rotation2d.fromDegrees(-120);
    public static final Rotation2d REEF_STATION_ID_21_ANGLE = Rotation2d.fromDegrees(180);
    public static final Rotation2d REEF_STATION_ID_22_ANGLE = Rotation2d.fromDegrees(120);

    // Red Reef Station ID Angle Constant //
    public static final Rotation2d REEF_STATION_ID_6_ANGLE = Rotation2d.fromDegrees(-60);
    public static final Rotation2d REEF_STATION_ID_7_ANGLE = Rotation2d.fromDegrees(0);
    public static final Rotation2d REEF_STATION_ID_8_ANGLE = Rotation2d.fromDegrees(60);
    public static final Rotation2d REEF_STATION_ID_9_ANGLE = Rotation2d.fromDegrees(120);
    public static final Rotation2d REEF_STATION_ID_10_ANGLE = Rotation2d.fromDegrees(180);
    public static final Rotation2d REEF_STATION_ID_11_ANGLE = Rotation2d.fromDegrees(-120);

    public static final Map<Integer, Rotation2d> REEF_STATION_ID_ANGLE_MAP = Map.ofEntries(
        Map.entry(6, REEF_STATION_ID_6_ANGLE),
        Map.entry(7, REEF_STATION_ID_7_ANGLE),
        Map.entry(8, REEF_STATION_ID_8_ANGLE),
        Map.entry(9, REEF_STATION_ID_9_ANGLE),
        Map.entry(10, REEF_STATION_ID_10_ANGLE),
        Map.entry(11, REEF_STATION_ID_11_ANGLE),
        Map.entry(17, REEF_STATION_ID_17_ANGLE),
        Map.entry(18, REEF_STATION_ID_18_ANGLE),
        Map.entry(19, REEF_STATION_ID_19_ANGLE),
        Map.entry(20, REEF_STATION_ID_20_ANGLE),
        Map.entry(21, REEF_STATION_ID_21_ANGLE),
        Map.entry(22, REEF_STATION_ID_22_ANGLE));

    // Reef station tag ID array //
    public static final int[] REEF_STATION_TAG_IDS = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    public static final int[] BLUE_REEF_STATION_TAG_IDS = { 17, 18, 19, 20, 21, 22 };
    public static final int[] RED_REEF_STATION_TAG_IDS = { 6, 7, 8, 9, 10, 11 };

    // The different layouts of the AprilTags on the field
    public static final AprilTagFieldLayout DEFAULT_APTAG_FIELD_LAYOUT;
    public static final AprilTagFieldLayout WELDED_RED_APTAG_FIELD_LAYOUT;
    public static final AprilTagFieldLayout WELDED_BLUE_APTAG_FIELD_LAYOUT;
    public static final AprilTagFieldLayout APTAG_FIELD_LAYOUT;

    // Static initializer block
    static {
      // Load default layout - this does NOT throw IOException
      // It might return null if the resource is missing, though kDefaultField should
      // be safe.
      AprilTagFieldLayout defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      if (defaultLayout == null) {
        System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
        DriverStation.reportError("CRITICAL: Failed to load default AprilTag field resource!", true);
        // defaultLayout remains null
      }

      // Initialize temporary variables for welded layouts
      AprilTagFieldLayout redLayout = null;
      AprilTagFieldLayout blueLayout = null;

      // Construct paths for welded layouts
      Path redPath = Path.of(
          Filesystem.getDeployDirectory().getPath(),
          "apriltags",
          "welded",
          "2025-red-no-barge.json");
      Path bluePath = Path.of(
          Filesystem.getDeployDirectory().getPath(),
          "apriltags",
          "welded",
          "2025-blue-no-barge.json");

      // Try loading layouts from file paths - THESE can throw IOException
      try {
        redLayout = new AprilTagFieldLayout(redPath);
        blueLayout = new AprilTagFieldLayout(bluePath);
      } catch (IOException e) {
        // Handle the error if loading from files fails
        System.err.println("!!! Failed to load welded AprilTag field layout files!");
        e.printStackTrace();
        DriverStation.reportError("Failed to load welded AprilTag layouts: " + e.getMessage(), true);
        // redLayout and blueLayout will remain null if they failed
      } finally {
        // Assign the loaded layouts to the final fields
        // Use default layout as fallback if welded layouts are still null
        DEFAULT_APTAG_FIELD_LAYOUT = defaultLayout;
        WELDED_RED_APTAG_FIELD_LAYOUT = (redLayout != null) ? redLayout : defaultLayout;
        WELDED_BLUE_APTAG_FIELD_LAYOUT = (blueLayout != null) ? blueLayout : defaultLayout;

        // Now initialize the main layout based on alliance
        AprilTagFieldLayout selectedLayout = DEFAULT_APTAG_FIELD_LAYOUT; // Start with default
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // Prefer loaded red layout, fallback to default
            selectedLayout = WELDED_RED_APTAG_FIELD_LAYOUT;
          } else { // Blue Alliance
            // Prefer loaded blue layout, fallback to default
            selectedLayout = WELDED_BLUE_APTAG_FIELD_LAYOUT;
          }
        } else {
          // Default if no alliance is set (e.g., practice mode) - use Blue or Default
          selectedLayout = WELDED_BLUE_APTAG_FIELD_LAYOUT;
        }

        // Assign the final selected layout
        APTAG_FIELD_LAYOUT = selectedLayout;

        // Final check if the main layout is still null (only if default also failed)
        if (APTAG_FIELD_LAYOUT == null) {
          // This is a critical state
          DriverStation.reportError("CRITICAL: No AprilTag field layout could be assigned!", true);
        }
      }
    }    
  }

  public static class VisionConstants {
    public static final String[] APTAG_CAMERA_NAMES = {
        "AprilTagAlignLeftCamera",
        "AprilTagAlignRightCamera",
        "AprilTagPoseEstCameraFL",
        "AprilTagPoseEstCameraFR",
        "AprilTagPoseEstCameraBL",
        "AprilTagPoseEstCameraBR"
    };

    // Main Apriltag alignment cam mounted facing forward, half a meter forward of
    // center, half a meter up from center.
    public static final Transform3d APTAG_ALIGN_LEFT_CAM_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(9.249),
            Units.inchesToMeters(4.910),
            Units.inchesToMeters(8.3885)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            0)
    );

    // Main Apriltag alignment cam mounted facing forward, half a meter forward of
    // center, half a meter up from center.
    public static final Transform3d APTAG_ALIGN_RIGHT_CAM_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(9.249),
            Units.inchesToMeters(-4.910),
            Units.inchesToMeters(8.3885)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            0)
    );

    // Front-Left Camera: Mounted at front-left corner, pointing outward at 30
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(45)
        )
    );

    // Front-Right Camera: Mounted at front-right corner, pointing outward at -30
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FR_POS = new Transform3d(
        new Translation3d(
          Units.inchesToMeters(17.125),
          Units.inchesToMeters(-17.125),
          Units.inchesToMeters(6.825)
      ),
      new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(-45)
        )
    );

    // Back-Left Camera: Mounted at back-left corner, pointing outward at 135
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(135)
        )
    );

    // Back-Right Camera: Mounted at back-right corner, pointing outward at -135
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BR_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)
        ),
        new Rotation3d(
            0,
            Units.degreesToRadians(-20),
            Units.degreesToRadians(-135)
        )
    );

    public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
        APTAG_POSE_EST_CAM_FL_POS,
        APTAG_POSE_EST_CAM_FR_POS,
        APTAG_POSE_EST_CAM_BL_POS,
        APTAG_POSE_EST_CAM_BR_POS
    };

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.1;
    public static double MAX_Z_ERROR = 0.5;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDDEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STDDEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STDDEV_FACTORS = new double[] {
        1.0, // APTAG_LEFT_CAM
        1.0, // APTAG_RIGHT_CAM
        1.0, // APTAG_POSE_EST_CAM_FL_POS
        1.0, // APTAG_POSE_EST_CAM_FR_POS
        1.0, // APTAG_POSE_EST_CAM_BL_POS
        1.0 // APTAG_POSE_EST_CAM_BR_POS
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY;; // More stable than full 3D
                                                                                           // solve

    // Vision range and aim PID constants //
    public static final double RANGE_P = 2;
    public static final double RANGE_I = 0;
    public static final double RANGE_D = 0;
    public static final double RANGE_TOLERANCE = 0.02;

    public static final double AIM_P = 2;
    public static final double AIM_I = 0;
    public static final double AIM_D = 0;
    public static final double AIM_TOLERANCE = 0.02;

    public static final double DESIRED_RANGE = 0.25;
    public static final double DESIRED_YAW_RIGHT = 0.03;
    public static final double DESIRED_YAW_LEFT = -0.03;
  }

  /** Intake Subsystem Constants */
  public static class CoralSubsystemConstants {
    public static final int MOTOR_ID = 1;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int FREE_CURRENT_LIMIT = 30;
    public static final int STALL_CURRENT_LIMIT = 50;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds

    public static final int BEAM_BREAK_1_DIGITAL_CHANNEL = 0;
    public static final int BEAM_BREAK_2_DIGITAL_CHANNEL = 1;
    public static final double CORAL_DETECT_CURRENT_THRESHOLD = 30.0;
    public static final double CORAL_DETECT_CANRANGE_THRESHOLD = 0.18;
    public static final double CORAL_DETECT_CANRANGE_HYSTERESIS = 0.01;

    /* Desired intake speed for intake and outtake */
    public static final double INTAKE_SPEED = 0.7;
    public static final double SLOW_INTAKE_SPEED = 0.15;
    public static final double OUTTAKE_SPEED = 1;
    public static final double REVERSE_SPEED = -0.5;
    public static final double SLOW_REVERSE_SPEED = -0.1;
  }

  public static class BoomstickSubsystemConstants {
    public static final int ARM_MOTOR_ID = 7;

    public static final double ARM_NOMINAL_VOLTAGE = 8;
    public static final int ARM_STALL_CURRENT_LIMIT = 20;
    public static final double ARM_SECONDARY_CURRENT_LIMIT = 30.0;
    public static final double ARM_RAMP_RATE_IN_SEC = 0.15; // Ramp rate in seconds

    public static final double ABS_ENC_OFFSET_VAL = 0.8810048; // @TODO: To be tuned later

    public static final ClosedLoopSlot PID_SLOT = ClosedLoopSlot.kSlot0;

    // Arm PID Constants //
    public static final double ARM_P = 1.0;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.0;
    public static final double ARM_F = 0.0;
    public static final double ARM_IZ = 0.0;
    public static final double ARM_MIN_OUTPUT = -1;
    public static final double ARM_MAX_OUTPUT = 1;

    // MaxMotion Constants //
    public static final double ARM_MAX_MAXMOTION_VELOCITY = 4000;
    public static final double ARM_MAX_MAXMOTION_ACCELERATION = 4000;
    public static final double ARM_MAXMOTION_ALLOWED_ERROR = 0.01;

    // Arm Physical Constants //
    public static final double ARM_REDUCTION = 12; // Mounted on 20:1 gear reduction
    public static final double ARM_MASS_KG = Units.lbsToKilograms(1.8);
    public static final double ARM_LENGTH_M = Units.inchesToMeters(22);

    // Algae Arm Feedforward Constants
    public static final double ARM_KS = 0.0; // TODO: To be tuned later
    public static final double ARM_KV = 0.762; // TODO: To be tuned later
    public static final double ARM_KA = 0.762; // TODO: To be tuned later
    public static final double ARM_KG = 0.0; // TODO: To be tuned later

    /*
     * Desired absolute encoder setpoint for Boomstick subsystem
     */
    public static final double ARM_HIGH_DEALGAE_GOAL = 0.24;
    public static final double ARM_LOW_DEALGAE_GOAL = 0.24;
    public static final double ARM_HOME_GOAL = 0.1;
  }

  public static class AlgaeSubsystemConstants {
    public static final int ARM_LEAD_MOTOR_ID = 2;
    public static final int ARM_FOLLOW_MOTOR_ID = 3;
    public static final int INTAKE_MOTOR_ID = 4;

    public static final double ARM_NOMINAL_VOLTAGE = 10.0;
    public static final int ARM_STALL_CURRENT_LIMIT = 40;
    public static final double ARM_SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double ARM_RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds

    public static final double INTAKE_NOMINAL_VOLTAGE = 10.0;
    public static final int INTAKE_FREE_CURRENT_LIMIT = 30;
    public static final int INTAKE_STALL_CURRENT_LIMIT = 50;
    public static final double INTAKE_SECONDARY_CURRENT_LIMIT = 60.0;

    public static final double ALGAE_DETECT_CURRENT_THRESHOLD = 40.0;

    public static final double ABS_ENC_OFFSET_VAL = 0.8810048; // @TODO: To be tuned later

    public static final ClosedLoopSlot PID_SLOT = ClosedLoopSlot.kSlot0;

    // Arm PID Constants //
    public static final double ARM_P = 1.0;
    public static final double ARM_I = 0.0;
    public static final double ARM_D = 0.0;
    public static final double ARM_F = 0.0;
    public static final double ARM_IZ = 0.0;
    public static final double ARM_MIN_OUTPUT = -1;
    public static final double ARM_MAX_OUTPUT = 1;

    // MaxMotion Constants //
    public static final double ARM_MAX_MAXMOTION_VELOCITY = 4000;
    public static final double ARM_MAX_MAXMOTION_ACCELERATION = 4000;
    public static final double ARM_MAXMOTION_ALLOWED_ERROR = 0.01;

    // Arm Physical Constants //
    public static final double ARM_REDUCTION = 20; // Mounted on 20:1 gear reduction
    public static final double ARM_MASS_KG = Units.lbsToKilograms(10.165);
    public static final double ARM_LENGTH_M = Units.inchesToMeters(17);

    // Algae Arm Feedforward Constants
    public static final double ARM_KS = 0.0; // TODO: To be tuned later
    public static final double ARM_KV = 0.762; // TODO: To be tuned later
    public static final double ARM_KA = 0.762; // TODO: To be tuned later
    public static final double ARM_KG = 0.0; // TODO: To be tuned later

    /*
     * Desired absolute encoder setpoint for moving shooter and amp mechanism (to be
     * tuned later using absolute encoder)
     */
    public static final double ARM_HOME_GOAL = 0.24;
    public static final double ARM_INTAKE_GOAL = 0.075;
    public static final double ARM_DEALGAE_GOAL = 0.1;
    public static final double ARM_HOLD_GOAL = 0.13;
    public static final double ARM_PROCESSOR_OUTTAKE_GOAL = 0.13;

    /* Desired intake speed for intake and outtake */
    public static final double INTAKE_SPEED = -1.0;
    public static final double DEALGAE_SPEED = 0.8;
    public static final double OUTTAKE_SPEED = 0.5;
  }

  /** Elevator Subsystem Constants */
  public static class ElevatorSubsystemConstants {
    public static final int LEAD_MOTOR_ID = 6;
    public static final int FOLLOW_MOTOR_ID = 5;

    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final int FREE_CURRENT_LIMIT = 40;
    public static final int STALL_CURRENT_LIMIT = 60;
    public static final double SECONDARY_CURRENT_LIMIT = 80.0;
    public static final double RAMP_RATE_IN_SEC = 0.1; // Ramp rate in seconds

    public static final double STARTING_LIMIT = -0.2;
    public static final double ENDING_LIMIT = -9.8;

    public static final ClosedLoopSlot PID_SLOT = ClosedLoopSlot.kSlot1;

    public static final double P = 0.4;
    public static final double I = 0.0;
    public static final double D = 0.1;
    public static final double F = 0.0;
    public static final double IZ = 0.0;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_OUTPUT = 1;

    // MaxMotion Constants //
    public static final double MAX_MAXMOTION_VELOCITY = 4000;
    public static final double MAX_MAXMOTION_ACCELERATION = 4000;
    public static final double MAXMOTION_ALLOWED_ERROR = 0.01;

    // Elevator Encoder Constants //
    public static final int AVERAGE_DEPTH = 64;
    public static final int COUNTS_PER_REVOLUTION = 4096;

    // Elevator Physical Constants //
    public static final double ELEVATOR_REDUCTION = 12; // Mounted on 12:1 gear reduction
    public static final double ELEVATOR_MASS_KG = Units.lbsToKilograms(20);

    // Elevator Feedforward Constants
    public static final double ELEVATOR_KS = 0.0;
    public static final double ELEVATOR_KV = 0.0;
    public static final double ELEVATOR_KA = 0.0;
    public static final double ELEVATOR_KG = 0.0;

    /*
     * Desired absolute encoder setpoint for moving elevator (to be tuned later
     * using relative encoder)
     */
    public static final double HOME = 0.0;
    public static final double L1 = -1;
    public static final double L2 = -3;
    public static final double L3 = -6;
    public static final double L4 = -10.4;
  }

  public static class LEDConstants {
    public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
    public static final double RAINBOW_PARTY_PALETTE = -0.97;
    public static final double RAINBOW_OCEAN_PALETTE = -0.95;
    public static final double RAINBOW_LAVE_PALETTE = -0.93;
    public static final double RAINBOW_FOREST_PALETTE = -0.91;
    public static final double RAINBOW_WITH_GLITTER = -0.89;
    public static final double CONFETTI = -0.87;
    public static final double SHOT_RED = -0.85;
    public static final double SHOT_BLUE = -0.83;
    public static final double SHOT_WHITE = -0.81;
    public static final double SINELON_RAINBOW_PALETTE = -0.79;
    public static final double SINELON_PARTY_PALETTE = -0.77;
    public static final double SINELON_OCEAN_PALETTE = -0.75;
    public static final double SINELON_LAVA_PALETTE = -0.73;
    public static final double SINELON_FOREST_PALETTE = -0.71;
    public static final double BPM_RAINBOW_PALETTE = -0.69;
    public static final double BPM_PARTY_PALETTE = -0.67;
    public static final double BPM_OCEAN_PALETTE = -0.65;
    public static final double BPM_LAVA_PALETTE = -0.63;
    public static final double BPM_FOREST_PALETTE = -0.61;
    public static final double FIRE_MEDIUM = -0.59;
    public static final double FIRE_LARGE = -0.57;
    public static final double TWINKLES_RAINBOW_PALETTE = -0.55;
    public static final double TWINKLES_PARTY_PALETTE = -0.53;
    public static final double TWINKLES_OCEAN_PALETTE = -0.51;
    public static final double TWINKLES_LAVA_PALETTE = -0.49;
    public static final double TWINKLES_FOREST_PALETTE = -0.47;
    public static final double COLOR_WAVES_RAINBOW_PALETTE = -0.45;
    public static final double COLOR_WAVES_PARTY_PALETTE = -0.43;
    public static final double COLOR_WAVES_OCEAN_PALETTE = -0.41;
    public static final double COLOR_WAVES_LAVA_PALETTE = -0.39;
    public static final double COLOR_WAVES_FOREST_PALETTE = -0.37;
    public static final double LARSON_SCANNER_RED = -0.35;
    public static final double LARSON_SCANNER_GRAY = -0.33;
    public static final double LIGHT_CHASE_RED = -0.31;
    public static final double LIGHT_CHASE_BLUE = -0.29;
    public static final double LIGHT_CHASE_GRAY = -0.27;
    public static final double HEARTBEAT_RED = -0.25;
    public static final double HEARTBEAT_BLUE = -0.23;
    public static final double HEARTBEAT_WHITE = -0.21;
    public static final double HEARTBEAT_GRAY = -0.19;
    public static final double BREATH_RED = -0.17;
    public static final double BREATH_BLUE = -0.15;
    public static final double BREATH_GRAY = -0.13;
    public static final double STROBE_RED = -0.11;
    public static final double STROBE_BLUE = -0.09;
    public static final double STROBE_GOLD = -0.07;
    public static final double STROBE_WHITE = -0.05;
    public static final double COLOR1_END_TO_END_BLEND_TO_BLACK = -0.03;
    public static final double COLOR1_LARSON_SCANNER = -0.01;
    public static final double COLOR1_LIGHT_CHASE = 0.01;
    public static final double COLOR1_HEARTBEAT_SLOW = 0.03;
    public static final double COLOR1_HEARTBEAT_MEDIUM = 0.05;
    public static final double COLOR1_HEARTBEAT_FAST = 0.07;
    public static final double COLOR1_BREATH_SLOW = 0.09;
    public static final double COLOR1_BREATH_FAST = 0.11;
    public static final double COLOR1_SHOT = 0.13;
    public static final double COLOR1_STROBE = 0.15;
    public static final double COLOR2_END_TO_END_BLEND_TO_BLACK = 0.17;
    public static final double COLOR2_LARSON_SCANNER = 0.19;
    public static final double COLOR2_LIGHT_CHASE = 0.21;
    public static final double COLOR2_HEARTBEAT_SLOW = 0.23;
    public static final double COLOR2_HEARTBEAT_MEDIUM = 0.25;
    public static final double COLOR2_HEARTBEAT_FAST = 0.27;
    public static final double COLOR2_BREATH_SLOW = 0.29;
    public static final double COLOR2_BREATH_FAST = 0.31;
    public static final double COLOR2_SHOT = 0.33;
    public static final double COLOR2_STROBE = 0.35;
    public static final double SPARKLE_1_ON_2 = 0.37;
    public static final double SPARKLE_2_ON_1 = 0.39;
    public static final double GRADIENT_1_AND_2 = 0.41;
    public static final double BPM_1_AND_2 = 0.43;
    public static final double END_TO_END_BLEND_1_TO_2 = 0.45;
    public static final double END_TO_END_BLEND_2_TO_1 = 0.47;
    public static final double COLOR_WAVES_1_AND_2 = 0.49;
    public static final double TWINKLES_1_AND_2 = 0.51;
    public static final double WAVES_1_AND_2 = 0.53;
    public static final double SINELON_1_AND_2 = 0.55;
    public static final double HOT_PINK = 0.57;
    public static final double DARK_RED = 0.59;
    public static final double RED = 0.61;
    public static final double RED_ORANGE = 0.63;
    public static final double ORANGE = 0.65;
    public static final double GOLD = 0.67;
    public static final double YELLOW = 0.69;
    public static final double LAWN_GREEN = 0.71;
    public static final double LIME = 0.73;
    public static final double DARK_GREEN = 0.75;
    public static final double GREEN = 0.77;
    public static final double BLUE_GREEN = 0.79;
    public static final double AQUA = 0.81;
    public static final double SKY_BLUE = 0.83;
    public static final double DARK_BLUE = 0.85;
    public static final double BLUE = 0.87;
    public static final double BLUE_VIOLET = 0.89;
    public static final double VIOLET = 0.91;
    public static final double WHITE = 0.93;
    public static final double GRAY = 0.95;
    public static final double DARK_GRAY = 0.97;
    public static final double BLACK = 0.99;
  }

  public static class SwerveConstants {
    // General constants for swerve drive //
    public static final double STEER_KP = 100;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0.5;
    public static final double STEER_KS = 0.1;
    public static final double STEER_KV = 1.59;
    public static final double STEER_KA = 0;

    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0.124;
    public static final double DRIVE_KA = 0;

    public static final double HEADING_KP = 3.5;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.5;
    public static final double HEADING_TOLERANCE = 0.01;

    public static final double ANGLE_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.12;
    public static final double PULSE_PER_ROTATION = 1;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
    public static final double MAX_SPEED_FEET_PER_SECOND = 18.2; // 18.2 feet per second
    public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND); // 18.2 feet
                                                                                                            // per
                                                                                                            // second

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
    public static final double CHASSIS_MASS = ROBOT_MASS;
    public static final Translation3d CHASSIS_CG = new Translation3d(0, 0, Units.inchesToMeters(8));
    public static final double LOOP_TIME = 0.13;

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10;

    public static final double SWERVE_DEADBAND = 0.1;

    // SWERVE MODULE ODOMETRY STANDARD DEVIATIONS //
    public static final Matrix<N3, N1> ODOMETRY_STD = VecBuilder.fill(0.1, 0.1, 0.1);
  }

  public static class OperatorControlNameConstants {
    public static final int ELEVATOR_L1_BTN = CustomButtonBoxConstants.BTN_4;
    public static final int ELEVATOR_L2_BTN = CustomButtonBoxConstants.BTN_3;
    public static final int ELEVATOR_L3_BTN = CustomButtonBoxConstants.BTN_2;
    public static final int ELEVATOR_L4_BTN = CustomButtonBoxConstants.BTN_1;
    public static final int ELEVATOR_HOME_BTN = CustomButtonBoxConstants.BTN_10;

    public static final int ALIGN_LEFT_CORAL_STATION_BTN = CustomButtonBoxConstants.BTN_5;
    public static final int ALIGN_RIGHT_CORAL_STATION_BTN = CustomButtonBoxConstants.BTN_6;

    public static final int ALIGN_LEFT_REEF_BRANCH_BTN = CustomButtonBoxConstants.BTN_11;
    public static final int ALIGN_RIGHT_REEF_BRANCH_BTN = CustomButtonBoxConstants.BTN_12;

    public static final int ALGAE_HOME_BTN = CustomButtonBoxConstants.BTN_10;
    public static final int INTAKE_ALGAE_BTN = CustomButtonBoxConstants.BTN_7;
    public static final int SCORE_ALGAE_BTN = CustomButtonBoxConstants.BTN_8;
    public static final int DEALGAE_BTN = CustomButtonBoxConstants.BTN_9;
  }

  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_BUTTON_PORT = 2;
    public static final int TEST_PORT = 3;
  }

  public static class CustomButtonBoxConstants {
    public static final int BTN_1 = 1;
    public static final int BTN_2 = 2;
    public static final int BTN_3 = 3;
    public static final int BTN_4 = 4;
    public static final int BTN_5 = 5;
    public static final int BTN_6 = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;
  }

  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }

  public static class Extreme3DProConstants {
    // Extreme 3D Pro Analog Axis/Stick //
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int ROTATE = 2;
    public static final int SLIDER = 3;

    // Extreme 3D Pro Buttons //
    public static final int BTN_TRIGGER = 1;
    public static final int BTN_THUMB = 2;
    public static final int BTN_BOT_LEFT = 3;
    public static final int BTN_BOT_RIGHT = 4;
    public static final int BTN_TOP_LEFT = 5;
    public static final int BTN_TOP_RIGHT = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;
  }
}
