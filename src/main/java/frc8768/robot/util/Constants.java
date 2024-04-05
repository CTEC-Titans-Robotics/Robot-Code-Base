package frc8768.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;

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
     * Field size in meters
     */
    public static final Translation2d FIELD_SIZE = new Translation2d(16.54175, 8.21055);

    public static final List<Integer> SPEAKER_IDS =
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? List.of(3, 4) : List.of(7,8);

    /**
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current motor type of swerve motors.
         */
        public static final MotorType CURRENT_TYPE = MotorType.TALONFX;

        /**
         * Max drive motor speed m/s.
         */
        public static final double MAX_SPEED = Units.feetToMeters(17); // enter feet per sec
        //Note: converts f/s to m/s
        //^^ units used by swerve lib

        public static  final double MAX_ROTATION = Units.degreesToRadians(460); //DEGREES PER SECOND
        //Note: converts degrees per second to radians per second
        //^^ units used by swerve lib
    }

    /**
     * Offsets in a 2D environment from an AprilTag based on ID
     */
    public enum FieldWaypoints {
        AMP(new Pose2d(new Translation2d(1.78, 7.72),
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
