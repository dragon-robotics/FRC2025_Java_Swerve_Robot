package frc.robot.subsystems.vision;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {}

    default void setTargetId(int id) {}
    
    class VisionIOInputs {
        /**
         * Last timestamp of the vision data for each camera
         */
        public double[] lastTimestamp = new double[]{0.0, 0.0};
        /**
         * Check if the camera has any apriltag targets
         */
        public boolean[] hasTargets = new boolean[]{false, false};
        /**
         * The target yaw and range for each camera for the tag with the closest id
         */
        public double[][] targetYawRange = new double[][]{{0.0, 0.0}, {0.0, 0.0}};
        /**
         * The closest apriltag id seen for each camera
         */
        public int[] tagId = new int[]{0, 0};
        /**
         * Target single tag id to track
         */
        public int targetTagId = 0;

    }
}
