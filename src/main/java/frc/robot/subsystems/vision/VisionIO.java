package frc.robot.subsystems.vision;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {}

    default void setTargetId(int id) {}
    
    class VisionIOInputs {
        /**
         * Are the cameras connected to the network?
         */
        public boolean leftCameraConnected;
        public boolean rightCameraConnected;
        /**
         * Last timestamp of the vision data for each camera
         */
        public double[] lastTimestamp = new double[]{0.0, 0.0};
        /**
         * Check if the camera has any apriltag targets
         */
        public boolean[] hasTargets = new boolean[]{false, false};
        /**
         * The target yaw and range for each camera for the tag with the best id
         */
        public double[][] targetYawAndRange = new double[][]{{-999.0, -999.0}, {-999.0, -999.0}};
        /**
         * The best apriltag id seen for each camera
         */
        public int[] tagId = new int[]{0, 0};
        /**
         * Target single tag id to track
         */
        public int targetTagId = 0;
        /**
         * Camera to use for vision tracking (true = left, false = right, default = right/left)
         */
        public boolean cameraToUse = false;
    }
}
