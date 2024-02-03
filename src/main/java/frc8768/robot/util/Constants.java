package frc8768.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

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
        public static final MotorType currentType = MotorType.SPARKMAX;

        /**
         * Max drive motor speed.
         */
        public static final double MAX_SPEED = 14.5;
    }

    public enum PoseToTagOffset {
        AMP(new Translation2d(0, -1.1D));
        public final Translation2d offsetVec;

        PoseToTagOffset(Translation2d offsetVec) {
            this.offsetVec = offsetVec;
        }

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
