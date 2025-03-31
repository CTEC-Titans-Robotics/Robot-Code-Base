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
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4*0.95);

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

    public enum DesiredPoses {
        TAG_8_RED(3.87, 2.99, Rotation2d.fromDegrees(60 - 180)),
        TAG_9_RED(5.09, 2.97, Rotation2d.fromDegrees(120 - 180)),
        TAG_19_BLUE(3.86, 4.95, Rotation2d.fromDegrees(120)),
        TAG_20_BLUE(5.09, 5.09, Rotation2d.fromDegrees(60));

        Pose2d desiredPose;

        DesiredPoses(double x, double y, Rotation2d rot) {
            desiredPose = new Pose2d(x, y, rot);
        }

        public Pose2d getDesiredPose() {
            return desiredPose;
        }

        public static DesiredPoses getClosest(Pose2d currPose) {
            DesiredPoses closest = null;
            for(DesiredPoses pose : DesiredPoses.values()) {
                if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red && pose.name().contains("RED")) {
                    if(closest == null) {
                        closest = pose;
                    } else if(currPose.getTranslation().getDistance(pose.desiredPose.getTranslation()) <
                            closest.desiredPose.getTranslation().getDistance(pose.desiredPose.getTranslation())) {
                        closest = pose;
                    }
                } else if(pose.name().contains("BLUE")) {
                    if(closest == null) {
                        closest = pose;
                    } else if(currPose.getTranslation().getDistance(pose.desiredPose.getTranslation()) <
                            closest.desiredPose.getTranslation().getDistance(pose.desiredPose.getTranslation())) {
                        closest = pose;
                    }
                }
            }

            return closest;
        }
    }
}
