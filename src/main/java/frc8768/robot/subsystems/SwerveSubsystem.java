package frc8768.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
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

/**
 * Container class for everything Swerve
 */
public class SwerveSubsystem {
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
                        null
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
                        null
                )
        );
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
        swerveDrive.drive(translation2d.times(Constants.SwerveConfig.MAX_SPEED), rotation * Math.toRadians(450), fieldRelative, isOpenLoop, pivotPoint);
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
        return new HashMap<>();
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
