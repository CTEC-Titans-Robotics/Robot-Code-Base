package frc8768.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.MathUtil;
import frc8768.robot.util.MotorType;
import swervelib.SwerveDrive;
import swervelib.imu.Pigeon2Swerve;
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
    private boolean isTortoise;

    /**
     * @param type Neos or Falcons, see {@link MotorType}
     * @throws IOException if it can't find the resources.
     */
    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case NEOS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive();
            case FALCONS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive();
        }
    }

    /**
     * Drive the motors
     *
     * @param translation2d X = Forward and back, Y = left and right.
     * @param rotation Rotation in Radians/Seconds
     * @param fieldRelative Use the Gyro as the permanent "front" of the Robot
     * @param isOpenLoop Don't use PID
     * @param headingCorrection Use heading correction.
     */
    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean headingCorrection) {
        swerveDrive.drive(translation2d.times(
                isTortoise ? swerveDrive.swerveDriveConfiguration.maxSpeed / 13.65 : swerveDrive.swerveDriveConfiguration.maxSpeed),
                isTortoise ? rotation * MathUtil.getRadFromDeg(225) : rotation * MathUtil.getRadFromDeg(450),
                fieldRelative, isOpenLoop, headingCorrection);
    }

    /**
     * Get the Gyro rotation in degrees.
     *
     * @return 3d rotation.
     */
    public Rotation3d getGyroRot() {
        return swerveDrive.getGyroRotation3d();
    }

    public void tortoiseMode() {
        isTortoise = true;
    }

    public void hareMode() {
        isTortoise = false;
    }

    public boolean isTortoise() {
        return isTortoise;
    }

    /**
     * Get the underlying instance.
     *
     * @return Underlying instance
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
