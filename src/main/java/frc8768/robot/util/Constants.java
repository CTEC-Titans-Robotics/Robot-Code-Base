package frc8768.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current motor type of swerve motors.
         */
        public static final MotorType CURRENT_TYPE = MotorType.TALONFX;

        /**
         * Max drive motor speed. m/s
         */
        public static final double MAX_SPEED = Units.feetToMeters(15.1);
    }

    /**
     * Offsets in a 2D environment from an AprilTag based on ID
     */
    public enum FieldWaypoints {
        AMP(new Pose2d(new Translation2d(1.75, 7.35),
                Rotation2d.fromDegrees(90))),
        SPEAKER(new Pose2d(new Translation2d(1.25, 5.52),
                Rotation2d.fromDegrees(0)));

        private Pose2d redPose;
        private Pose2d bluePose;

        FieldWaypoints(Pose2d bluePose) {
            this.bluePose = bluePose;
            this.redPose = new Pose2d(new Translation2d(FIELD_SIZE.getX() - bluePose.getX(), bluePose.getY()),
                    bluePose.getRotation().times(-1));
        }

        /**
         * Get the Desired Field pose near a field element
         * @return The desired Pose2d
         */
        public Pose2d getPose2d() {
            if(DriverStation.getAlliance().isPresent()) {
                return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? bluePose : redPose;
            }
            return this.bluePose;
        }
    }
}
