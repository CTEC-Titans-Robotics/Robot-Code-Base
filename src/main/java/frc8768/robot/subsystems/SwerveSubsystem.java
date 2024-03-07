package frc8768.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.Robot;
import frc8768.robot.util.Constants;
import frc8768.robot.util.MathUtil;
import frc8768.robot.util.MotorType;
import frc8768.robot.util.Constants;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.PhotonVision;
import frc8768.visionlib.Vision;
import frc8768.visionlib.helpers.LimelightHelpers;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Container class for everything Swerve
 */
public class SwerveSubsystem implements Subsystem {
    /**
     * The underlying YAGSL implementation
     */
    private SwerveDrive swerveDrive;
    /**
     * A Thread that updates the swerve odometry based on a apriltag
     */
    private final VisionOdomThread visionUpdateThread;

    /**
     * @param type NEOs or Falcons, see {@link MotorType}
     * @throws IOException if it can't find the resources.
     */
    public SwerveSubsystem(MotorType type) throws IOException {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation((double) 150/7, 1);
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);

        switch(type) {
            case SPARKMAX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, angleConversionFactor, driveConversionFactor);
            case TALONFX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/talonfx")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, angleConversionFactor, driveConversionFactor);
            case SPARKFLEX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/sparkflex")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, angleConversionFactor, driveConversionFactor);
        }

        this.visionUpdateThread = new VisionOdomThread(this, Robot.getInstance().getLeftVision(), "VisionOdom Thread");
        this.visionUpdateThread.start();
    }

    /**
     * Drive the motors
     *
     * @param translation2d X = Forward and back, Y = left and right.
     * @param rotation Rotation in Radians/Seconds
     * @param fieldRelative Use the Gyro as the permanent "front" of the Robot
     * @param isOpenLoop Don't use PID
     * @param pivotPoint 2d Pivot point for rotation
     */
    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop, Translation2d pivotPoint) {
        swerveDrive.drive(translation2d.times(Constants.SwerveConfig.MAX_SPEED), rotation * MathUtil.getRadFromDeg(450), fieldRelative, isOpenLoop, pivotPoint);
    }

    /**
     * Get the Gyro rotation in degrees.
     *
     * @return 3d rotation.
     */
    public Rotation3d getGyroRot() {
        return swerveDrive.getGyroRotation3d();
    }

    /**
     * Get the underlying instance.
     *
     * @return Underlying instance
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Dashboard logging
     *
     * @return Map of Name to Value
     */
    public Map<String, String> dashboard() {
        HashMap<String, String> map = new HashMap<>();
        for(int i = 0; i < 4; i++) {
            map.put(String.format("Module %d Drive: Velocity", i),
                    String.valueOf(swerveDrive.getModules()[i].getDriveMotor().getVelocity()));
            map.put(String.format("Module %d Angle: Velocity", i),
                    String.valueOf(swerveDrive.getModules()[i].getAngleMotor().getVelocity()));

            map.put(String.format("Module %d Drive: Position", i),
                    String.valueOf(swerveDrive.getModules()[i].getDriveMotor().getPosition()));
            map.put(String.format("Module %d Angle: Position", i),
                    String.valueOf(swerveDrive.getModules()[i].getAngleMotor().getPosition()));
        }
        return map;
    }

    /**
     * Log string, (Could use a buffer here to prevent data race)
     *
     * @return List of different Strings.
     */
    public List<String> log() {
        ArrayList<String> list = new ArrayList<>();
        // Insert string buffer, different logic for detecting errors here
        return list;
    }

    public static class VisionOdomThread extends Thread {
        private SwerveSubsystem swerve;
        private PhotonVision vision;

        public VisionOdomThread(SwerveSubsystem swerve, PhotonVision vision, String name) {
            super(name);
            this.swerve = swerve;
            this.vision = vision;
        }

        @Override
        public void run() {
            int i = 0;
            while(true) {
                i++;
                if(i < 500000) {
                    continue;
                } else if(i > 500000) {
                    i = 0;
                }
                PhotonPipelineResult target = this.vision.getBestTarget();
                if(target == null)
                    continue;

                Pose3d pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestTarget().getBestCameraToTarget(),
                        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo).getTagPose(target.getBestTarget().getFiducialId()).get(),
                        new Transform3d(-Units.inchesToMeters(5.25), Units.inchesToMeters(10.75), Units.inchesToMeters(19.25), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(180))));
                this.swerve.getSwerveDrive().addVisionMeasurement(pose.toPose2d(), target.getTimestampSeconds());
            }
        }
    }
}
