package frc8768.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader;

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

    public static final Pathfinder PATHFINDER = new PathfinderBuilder(FieldLoader.Field.CRESCENDO_2024)
            .setRobotWidth(0.66)
            .setRobotLength(0.66)
            .build();

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
        AMP(new Translation2d(0, -1.1D)),
        SPEAKER(new Translation2d(0.11, -1.24D));
        public final Translation2d offsetVec;

        PoseToTagOffset(Translation2d offsetVec) {
            this.offsetVec = offsetVec;
        }

        public static PoseToTagOffset getTagOffsetsForId(int id) {
            switch(id) {
                case 5, 6 -> {
                    return AMP;
                }
                case 4, 7 -> {
                    return SPEAKER;
                }
            }
            return null;
        }
    }
}
