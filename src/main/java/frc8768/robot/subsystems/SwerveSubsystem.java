package frc8768.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.Robot;
import frc8768.robot.util.Constants;
import frc8768.robot.util.MathUtil;
import frc8768.robot.util.MotorType;
import frc8768.visionlib.PhotonVision;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;

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
        swerveDrive.drive(translation2d.times(Constants.SwerveConfig.MAX_SPEED), rotation * MathUtil.getRadFromDeg(250), fieldRelative, isOpenLoop, pivotPoint);
    }

    public void autonInit() {
        // this.visionUpdateThread.start();
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
            while(true) {
                if(Robot.getInstance().isTeleopEnabled()) {
                    continue;
                }
                PhotonPipelineResult target = this.vision.getBestTarget();
                if(target == null)
                    continue;
                if(!target.hasTargets())
                    continue;

                Pose3d pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestTarget().getBestCameraToTarget(),
                        AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
                                .getTagPose(target.getBestTarget().getFiducialId()).get(),
                        new Transform3d(Units.inchesToMeters(5.25), Units.inchesToMeters(10.75), -Units.inchesToMeters(19.25),
                                new Rotation3d(Units.degreesToRadians(-30), Units.degreesToRadians(0), Units.degreesToRadians(0))));
                this.swerve.getSwerveDrive().addVisionMeasurement(pose.toPose2d(), target.getTimestampSeconds());
            }
        }
    }
}
