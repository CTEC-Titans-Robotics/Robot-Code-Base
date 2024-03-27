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

    public static final Translation2d FIELD_SIZE = new Translation2d(16.54175, 8.21055);

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
        public static final double MAX_SPEED = Units.feetToMeters(14.5);  //enter feet
        // | note: all wpi calculations use meters to conversion is done here

        public static final double MAX_ROTATION = 600;  //degrees
    }

    public enum FieldWaypoints {
        AMP(new Pose2d(new Translation2d(1.75, 7.35),
                            Rotation2d.fromDegrees(90))),
        SPEAKER(new Pose2d(new Translation2d(1.25, 5.52),
                            Rotation2d.fromDegrees(0)));

        private Pose2d redPose;
        private Pose2d bluePose;

        FieldWaypoints(Pose2d bluePose) {
            this.bluePose = bluePose;
            this.redPose = new Pose2d(new Translation2d(FIELD_SIZE.getX() - bluePose.getX(), bluePose.getY()), bluePose.getRotation());
        }

        public Pose2d getPose2d() {
            if(DriverStation.getAlliance().isPresent()) {
                return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? bluePose : redPose;
            }
            return this.bluePose;
        }
    }
}
