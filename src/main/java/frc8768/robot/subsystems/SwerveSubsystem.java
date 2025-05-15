package frc8768.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc8768.robot.util.Constants;
import frc8768.robot.util.MotorType;
import frc8768.visionlib.helpers.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.*;

import static edu.wpi.first.units.Units.DegreesPerSecond;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Container class for everything Swerve
 */
public class SwerveSubsystem implements Subsystem {
    /**
     * SysID routine for the Drive motor of module 0
     */
    private final SysIdRoutine driveSysIdRoutine;

    /**
     * SysID routine for the Angle motor of module 0
     */
    private final SysIdRoutine angleSysIdRoutine;



    ////
    // Holonomic controller components
    private final PIDController xController = new PIDController(1.5, 0.0, 0.0);
    private final PIDController yController = new PIDController(1.5, 0.0, 0.0);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            3.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(Math.toRadians(360), Math.toRadians(720))
    );
    private final HolonomicDriveController holonomicController;
    private Pose2d targetPose = null;
    ////



    /**
     * The underlying YAGSL implementation
     */
    private SwerveDrive swerveDrive;

    private Rotation2d initialYaw;

    /**
     * @param type Neos or Falcons, see {@link MotorType}
     * @throws IOException if it can't find the resources.
     */
    public SwerveSubsystem(MotorType type) throws IOException {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        holonomicController = new HolonomicDriveController(xController, yController, thetaController);

        double metersPerRotation = SwerveMath.calculateMetersPerRotation(Constants.SwerveConfig.WHEEL_DIAMETER, Constants.SwerveConfig.DRIVE_GEAR_RATIO);
        double metersPerDeg = SwerveMath.calculateDegreesPerSteeringRotation(Constants.SwerveConfig.TURN_GEAR_RATIO);

        switch(type) {
            case SPARKMAX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, metersPerDeg, metersPerRotation);
            case TALONFX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, metersPerDeg, metersPerRotation);
            case SPARKFLEX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/sparkflex")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED, metersPerDeg, metersPerRotation);
        }

        initialYaw = Rotation2d.fromDegrees(swerveDrive.getYaw().getDegrees());

        driveSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        volt -> swerveDrive.getModules()[0].getDriveMotor().setVoltage(volt.in(Units.Volts)),
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("drive-motor")
                                    .voltage(Units.Volts.of(swerveDrive.getModules()[0].getDriveMotor().getVoltage()))
                                    .linearPosition(Units.Meter.of(swerveDrive.getModules()[0].getDriveMotor().getPosition()))
                                    .linearVelocity(
                                            Units.MetersPerSecond.of(swerveDrive.getModules()[0].getDriveMotor().getVelocity()));
                        },
                        this
                )
        );

        angleSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        volt -> swerveDrive.getModules()[0].getAngleMotor().setVoltage(volt.in(Units.Volts)),
                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("angle-motor")
                                    .voltage(Units.Volts.of(swerveDrive.getModules()[0].getAngleMotor().getVoltage()))
                                    .angularPosition(Units.Degree.of(swerveDrive.getModules()[0].getAbsolutePosition()))
                                    .angularVelocity(
                                            DegreesPerSecond.of(swerveDrive.getModules()[0].getAngleMotor().getVelocity()));
                        },
                        this
                )
        );

        swerveDrive.setHeadingCorrection(true);
        // swerveDrive.setCosineCompensator(true);
         swerveDrive.setAngularVelocityCompensation(true, true, 0.15);
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
        swerveDrive.drive(translation2d.times(Constants.SwerveConfig.MAX_SPEED), rotation * Constants.SwerveConfig.MAX_ROTATION_SPEED, fieldRelative, isOpenLoop, pivotPoint);
    }

    public void rotate(double rotationSpeed){
        swerveDrive.drive(new Translation2d(0,0),rotationSpeed, false, false, Constants.BOT_CENTER);
    }
    public void move(double xSpeed, double ySpeed, double rot) {
        drive(new Translation2d(xSpeed, ySpeed), rot, false, false, Constants.BOT_CENTER);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();

        initialYaw = swerveDrive.getYaw();
    }
    double needsRotation;
    public void setTargetHeading(Translation2d translation, double target, double status) {
        needsRotation = status;
        Pose2d currPose = swerveDrive.getPose();
        if(MathUtil.isNear(target, currPose.getRotation().getDegrees(), 2)) {
            drive(translation, 0, true, false, Constants.BOT_CENTER);
            needsRotation=0;
            return;
        }
        if(needsRotation == 1 && target - currPose.getRotation().getDegrees() > 2) {
            drive(translation, 0.2, true, false, Constants.BOT_CENTER);
        }
        if(needsRotation == 1 && target - currPose.getRotation().getDegrees() < 2) {
            drive(translation, -0.2, true, false, Constants.BOT_CENTER);
        }
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
    public Map<String, Object> dashboard() {
        HashMap<String, Object> encoder = new HashMap<>();

        encoder.put("frontleft", swerveDrive.getModules()[0].getAbsolutePosition());
        encoder.put("frontright", swerveDrive.getModules()[1].getAbsolutePosition());
        encoder.put("backleft", swerveDrive.getModules()[2].getAbsolutePosition());
        encoder.put("backright", swerveDrive.getModules()[3].getAbsolutePosition());

        return encoder;
    }

    /**
     * Get the command for SysID Drive
     *
     * @return The associated command
     */
    public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.quasistatic(direction);
    }

    /**
     * Get the command for SysID Angle
     *
     * @return The associated command
     */
    public Command sysIdQuasistaticAngle(SysIdRoutine.Direction direction) {
        return angleSysIdRoutine.quasistatic(direction);
    }

    /**
     * Get the command for SysID Drive
     *
     * @return The associated command
     */
    public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.dynamic(direction);
    }

    /**
     * Get the command for SysID Angle
     *
     * @return The associated command
     */
    public Command sysIdDynamicAngle(SysIdRoutine.Direction direction) {
        return angleSysIdRoutine.dynamic(direction);
    }

    /**
     * Log string, (Could use a buffer here to prevent data race)
     *
     * @return List of different Strings.
     */
    public List<String> log() {
        // Insert string buffer/different logic for detecting faults here
        return new ArrayList<>();
    }


    public void setTargetPose(Pose2d target) {
        this.targetPose = target;
    }

    Optional<LimelightHelpers.PoseEstimate> mt2BACK = Optional.empty();
    List<Integer> reefIds = List.of(6,7,8,9,10,11,17,18,19,20,21,22);

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        ////
        // Drive toward target pose if one is set
        if (targetPose != null) {
            Pose2d currentPose = swerveDrive.getPose();

            Trajectory.State goalState = new Trajectory.State(
                    0.0,                          // timeSeconds (not used here)
                    0.0,                          // velocity (optional)
                    0.0,                          // acceleration (optional)
                    targetPose,                   // your target pose
                    0.0                           // curvature (optional)
            );

            ChassisSpeeds speeds = holonomicController.calculate(
                    swerveDrive.getPose(),
                    goalState,
                    targetPose.getRotation()
            );


            ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    currentPose.getRotation()
            );

            swerveDrive.drive(fieldRelativeSpeeds);

            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

            if (distance < 0.05 && angleError < 5.0) {
                targetPose = null;
                swerveDrive.drive(new ChassisSpeeds());
            }
        }

        if(DriverStation.getAlliance().isPresent()) {
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if(alliance == DriverStation.Alliance.Red) {
                LimelightHelpers.SetRobotOrientation("limelight-back", swerveDrive.getYaw().getDegrees() - 180, swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond),0,0,0,0);
                LimelightHelpers.SetRobotOrientation("limelight-front", swerveDrive.getYaw().getDegrees() - 180, swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond),0,0,0,0);
                if(LimelightHelpers.getTargetCount("limelight-back") > 0){
                    mt2BACK = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-back"));
                }
            } else {
                LimelightHelpers.SetRobotOrientation("limelight-back", swerveDrive.getYaw().getDegrees(), swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond),0,0,0,0);
                LimelightHelpers.SetRobotOrientation("limelight-front", swerveDrive.getYaw().getDegrees(), swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond),0,0,0,0);
                if(LimelightHelpers.getTargetCount("limelight-back") > 0){
                    mt2BACK = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back"));
                }
            }
        }

        //Post esitamtion from vision, only use if < 2 meters

        //SmartDashboard.putNumber("LL Swerve X", mt2BACK.pose.getX());
        //SmartDashboard.putNumber("LL Swerve Y", mt2BACK.pose.getY());
        //SmartDashboard.putNumber("LL Swerve Rot", mt2BACK.pose.getRotation().getDegrees());
        //SmartDashboard.putNumber("LL Tag Dist", mt2BACK.avgTagDist);

        mt2BACK.ifPresent((poseEstimate) -> {
            if (poseEstimate.pose.getX() != 0 && poseEstimate.pose.getY() != 0 && poseEstimate.pose.getRotation().getDegrees() != 0) {
                //SmartDashboard.putNumber("LL Tag ID", mt2BACK.rawFiducials[0].id);
                if (poseEstimate.avgTagDist < 2 && reefIds.contains(poseEstimate.rawFiducials[0].id) && DriverStation.isTeleop()) {
                    swerveDrive.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                }
            }
        });
    }
}
