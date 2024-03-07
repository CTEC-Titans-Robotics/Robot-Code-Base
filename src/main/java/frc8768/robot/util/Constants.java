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

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    public static final double targetWidth =
            Units.inchesToMeters(41.30) - Units.inchesToMeters(6.5); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight =
            Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos =
            Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final double kFarTgtZPos =
            (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    /**
     * Swerve-specific configuration.
     */
    public static class SwerveConfig {
        /**
         * Current motor type of swerve motors.
         */
        public static final MotorType CURRENT_TYPE = MotorType.TALONFX;

        /**
         * Max drive motor speed m/s. m/s
         */
        public static final double MAX_SPEED = Units.feetToMeters(17);
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
