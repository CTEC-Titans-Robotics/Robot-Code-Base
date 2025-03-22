package frc8768.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc8768.robot.util.Constants;
import frc8768.robot.util.MotorType;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

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
                                            Units.DegreesPerSecond.of(swerveDrive.getModules()[0].getAngleMotor().getVelocity()));
                        },
                        this
                )
        );

        swerveDrive.setHeadingCorrection(true);
        // swerveDrive.setCosineCompensator(true);
        // swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
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

    public void move(double xSpeed, double ySpeed, double rot) {
        drive(new Translation2d(xSpeed, ySpeed), rot, false, false, Constants.BOT_CENTER);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();

        initialYaw = swerveDrive.getYaw();
    }

    public void setTargetHeading(Translation2d translation, double target) {
        Pose2d currPose = swerveDrive.getPose();
        if(MathUtil.isNear(target, currPose.getRotation().getDegrees(), 2)) {
            drive(translation, 0, true, false, Constants.BOT_CENTER);
            return;
        }
        drive(translation, target > currPose.getRotation().getDegrees() ? 0.2 : -0.2, true, false, Constants.BOT_CENTER);
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


}
