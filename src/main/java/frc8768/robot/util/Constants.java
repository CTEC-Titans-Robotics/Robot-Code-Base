package frc8768.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Anything that stays the same throughout the runtime of the program should be in here.
 */
public class Constants {
    /**
     * The Main Driver controller ID
     */
    public static final int DRIVER_CONTROLLER_ID = 0;

    /**
     * Controller deadband, prevents accidental input
     */
    public static final double CONTROLLER_DEADBAND = 0.05;

    /**
     * Center of bot
     */
    public static final Translation2d BOT_CENTER = new Translation2d(0, 0);

    /**
     * Field size in meters, relative to 0,0
     */
    public static final Translation2d FIELD_SIZE = new Translation2d(16.54175, 8.21055);

    /**
     * Weight of Robot in Kilograms
     */
    public static final double WEIGHT = 120;

    /**
     * Inertia of Momentum in KG Sq Meters
     */
    public static final double INERTIA = 4;

    /**
     * How often to poll operators, ms
     */
    public static final int POLL_RATE = 20;

    /**
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current motor type of swerve motors
         */
        public static final MotorType CURRENT_TYPE = MotorType.TALONFX;

        /**
         * Max drive motor speed, m/s
         */
        public static final double MAX_SPEED = Units.feetToMeters(19.5);

        /**
         * Output wheel diameter in meters
         */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

        /**
         * Drive gear ratio from motor to output shaft
         */
        public static final double DRIVE_GEAR_RATIO = 5.36;

        /**
         * Turn gear ratio from motor to output shaft
         */
        public static final double TURN_GEAR_RATIO = 150/7D;

        /**
         * Max rotation speed
         */
        public static final double MAX_ROTATION_SPEED = Math.toRadians(450);
    }
}
