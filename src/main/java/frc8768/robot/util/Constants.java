package frc8768.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc8768.visionlib.Vision;

public class Constants {
    /**
     * The Main Driver controller ID
     */
    public static final int driverControllerId = 0;

    /**
     * The Co-Driver controller ID
     */
    public static final int coDriverControllerId = 1;

    /**
     * Controller deadband, prevents accidental input
     */
    public static final double controllerDeadband = 0.1;

    /**
     * Center of bot
     */
    public static final Translation2d BOT_CENTER = new Translation2d(0, 0);

    /**
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current type of swerve motors.
         */
        public static final MotorType currentType = MotorType.TALONFX;

        /**
         * Max drive motor speed ft/s.
         */
        public static final double MAX_SPEED = 14.5;
    }

    /**
     * Offsets in a 2D environment from an AprilTag based on ID
     */
    public enum PoseToTagOffset {
        /**
         * Example Amp offset.
         */
        AMP(new Translation2d(0, -1.1D));

        /**
         * 2-dimensional vector.
         */
        public final Translation2d offsetVec;

        /**
         * Default constructor
         * @param offsetVec 2-sized array with 2d dimensions from the AprilTag
         */
        PoseToTagOffset(Translation2d offsetVec) {
            this.offsetVec = offsetVec;
        }

        /**
         * Get the 2D offsets from an AprilTag ID
         * @param id The ID of the AprilTag, see {@link Vision#getTargetID()}
         * @return An instance of PoseToTagOffset. Holds a 2-dimensional vector.
         */
        public static PoseToTagOffset getTagOffsetsForId(int id) {
            switch(id) {
                case 5, 6 -> {
                    return AMP;
                }
            }
            return null;
        }
    }
}
