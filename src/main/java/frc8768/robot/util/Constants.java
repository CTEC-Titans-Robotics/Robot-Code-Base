package frc8768.robot.util;

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
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current type of swerve motors.
         */
        public static final MotorType currentType = MotorType.NEOS;
    }
}
