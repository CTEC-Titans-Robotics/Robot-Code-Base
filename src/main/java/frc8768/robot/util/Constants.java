package frc8768.robot.util;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.sql.Driver;

import static edu.wpi.first.units.Units.*;

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

    public static final double ROBOT_WIDTH = Units.inchesToMeters(32);

    /**
     * Weight of Robot in Kilograms
     */
    public static final double WEIGHT = Units.lbsToKilograms(107.5);

    /**
     * Inertia of Momentum in KG Sq Meters
     */
    public static final double INERTIA = 0.2;

    /**
     * How often to poll operators, ms
     */
    public static final int POLL_RATE = 50;

    public static final Command DEFAULT_COMMAND = new InstantCommand();
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
            MetersPerSecond.of(3),
            MetersPerSecondPerSecond.of(1.5),
            RadiansPerSecond.of(6),
            RadiansPerSecondPerSecond.of(6),
            Volts.of(12),
            false
    );

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
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4*0.97);

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
    static double robotDistToCenter = 16;
    static double xAdjust = Units.inchesToMeters(robotDistToCenter/2);  //robot width/2
    static double yAdjust = Units.inchesToMeters((robotDistToCenter/2)*Math.sqrt(3));  //robot width/2*sqrt(3)
    public enum TagLocations {
        //CENTER
        TAG_8_RED(3.887703, 2.97868, Rotation2d.fromDegrees(240)),
        TAG_17_BLUE(3.870706, 2.954368, Rotation2d.fromDegrees(240)),
        TAG_7_RED(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_18_BLUE(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_6_RED(3.870706, 5.097432, Rotation2d.fromDegrees(120)),

        TAG_19_BLUE(3.870706, 5.097432, Rotation2d.fromDegrees(120)),
        TAG_11_RED(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_20_BLUE(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_10_RED(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_21_BLUE(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_9_RED(5.10794, 2.954368, Rotation2d.fromDegrees(300)),
        TAG_22_BLUE(5.10794, 2.954368, Rotation2d.fromDegrees(300)),

        //False = LEFT   x => 0.17598, y=> 0.1016
        TAG_8_RED_FALSE(3.711723, 3.08088, Rotation2d.fromDegrees(240)),
        TAG_17_BLUE_FALSE(3.870706, 2.954368, Rotation2d.fromDegrees(240)),
        TAG_7_RED_FALSE(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_18_BLUE_FALSE(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_6_RED_FALSE(3.870706, 5.097432, Rotation2d.fromDegrees(120)),
        TAG_19_BLUE_FALSE(3.870706, 5.097432, Rotation2d.fromDegrees(120)),
        TAG_11_RED_FALSE(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_20_BLUE_FALSE(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_10_RED_FALSE(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_21_BLUE_FALSE(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_9_RED_FALSE(5.10794, 2.954368, Rotation2d.fromDegrees(300)),
        TAG_22_BLUE_FALSE(5.10794, 2.954368, Rotation2d.fromDegrees(300)),


        //TRUE = Right    x => .11, y => 0.0635
        TAG_8_RED_TRUE(3.997683, 2.91578, Rotation2d.fromDegrees(240)),
        TAG_17_BLUE_TRUE(3.870706, 2.954368, Rotation2d.fromDegrees(240)),
        TAG_7_RED_TRUE(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_18_BLUE_TRUE(3.2511, 4.0259000, Rotation2d.fromDegrees(180)),
        TAG_6_RED_TRUE(3.870706, 5.097432, Rotation2d.fromDegrees(120)),
        TAG_19_BLUE_TRUE(3.870706, 5.097432, Rotation2d.fromDegrees(120)),
        TAG_11_RED_TRUE(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_20_BLUE_TRUE(5.10794, 5.097432, Rotation2d.fromDegrees(60)),
        TAG_10_RED_TRUE(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_21_BLUE_TRUE(5.727446, 4.0259000, Rotation2d.fromDegrees(0)),
        TAG_9_RED_TRUE(5.10794, 2.954368, Rotation2d.fromDegrees(300)),
        TAG_22_BLUE_TRUE(5.10794, 2.954368, Rotation2d.fromDegrees(300));

        final Pose2d tagPose;

        TagLocations(double tagX, double tagY, Rotation2d tagRot) {
            tagPose = new Pose2d(tagX, tagY, tagRot);
        }

        public Pose2d getDesiredPose(double robotRelativeX, double robotRelativeY, Rotation2d robotRelativeRot) {
            ///NEW
/*
            double xAdjust = 0.37395;  //robot width/2
            double yAdjust = 0.10795;  //robot width/2*sqrt(3)
            Pose2d robotPose = new Pose2d();
            if(tagPose.getRotation().getDegrees() == 0){robotPose = new Pose2d(tagPose.getX()+xAdjust,tagPose.getY(),tagPose.getRotation());}
            else if(tagPose.getRotation().getDegrees() == 60){robotPose = new Pose2d(tagPose.getX()+xAdjust,tagPose.getY()+yAdjust,tagPose.getRotation());}
            else if(tagPose.getRotation().getDegrees() == 120){robotPose = new Pose2d(tagPose.getX()-xAdjust,tagPose.getY()+yAdjust,tagPose.getRotation());}
            else if(tagPose.getRotation().getDegrees() == 180){robotPose = new Pose2d(tagPose.getX()-xAdjust,tagPose.getY(),tagPose.getRotation());}
            else if(tagPose.getRotation().getDegrees() == 240){robotPose = new Pose2d(tagPose.getX()+xAdjust,tagPose.getY()-yAdjust,tagPose.getRotation());}
            else if(tagPose.getRotation().getDegrees() == 300){robotPose = new Pose2d(tagPose.getX()-xAdjust,tagPose.getY()-yAdjust,tagPose.getRotation());}

            Translation2d offsetRotated = new Translation2d(robotRelativeX, robotRelativeY).rotateBy(tagPose.getRotation());
            return robotPose.plus(new Transform2d(offsetRotated, robotRelativeRot));
///            return robotPose;
*/
            ///OLD
            Translation2d offsetRotated = new Translation2d(robotRelativeX, robotRelativeY).rotateBy(tagPose.getRotation());
            return tagPose.plus(new Transform2d(offsetRotated, robotRelativeRot));
        }

        public static TagLocations getClosest(Pose2d currPose, String alignment) {
            TagLocations closest = null;
            for(TagLocations pose : TagLocations.values()) {
                if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red && pose.name().contains("RED")&& pose.name().contains(alignment)) {
                    if(closest == null) {
                        closest = pose;
                    } else if(currPose.getTranslation().getDistance(pose.tagPose.getTranslation()) <
                            closest.tagPose.getTranslation().getDistance(pose.tagPose.getTranslation())) {
                        closest = pose;
                    }
                } else if(pose.name().contains("BLUE")&& pose.name().contains(alignment)) {
                    if(closest == null) {
                        closest = pose;
                    } else if(currPose.getTranslation().getDistance(pose.tagPose.getTranslation()) <
                            closest.tagPose.getTranslation().getDistance(pose.tagPose.getTranslation())) {
                        closest = pose;
                    }
                }
            }

            return closest;
        }
    }
}
