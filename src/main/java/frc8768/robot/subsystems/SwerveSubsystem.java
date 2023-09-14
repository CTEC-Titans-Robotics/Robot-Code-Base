package frc8768.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.MotorType;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;

/*
    Subsystems are NOT threaded to avoid a race condition with their operators.
 */
public class SwerveSubsystem implements Subsystem {
    private SwerveDrive swerveDrive;
    private boolean isTortoise;

    private final double hareSpeed;
    private final double hareAngularVelocity;

    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case NEOS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive();
            case FALCONS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive();
        }

        hareSpeed = swerveDrive.swerveDriveConfiguration.maxSpeed;
        hareAngularVelocity = swerveDrive.swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond;
    }

    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean headingCorrection) {
        swerveDrive.drive(translation2d.times(
                swerveDrive.swerveDriveConfiguration.maxSpeed),
                rotation * swerveDrive.swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond,
                fieldRelative, isOpenLoop, headingCorrection);
    }

    public Rotation3d getGyroRot() {
        return swerveDrive.getGyroRotation3d();
    }

    public void tortoiseMode() {
        isTortoise = true;
        swerveDrive.swerveDriveConfiguration.maxSpeed = 0.75;
        swerveDrive.swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond = 1;
    }

    public void hareMode() {
        isTortoise = false;
        swerveDrive.swerveDriveConfiguration.maxSpeed = hareSpeed;
        swerveDrive.swerveDriveConfiguration.attainableMaxRotationalVelocityRadiansPerSecond = hareAngularVelocity;
    }

    public boolean isTortoise() {
        return isTortoise;
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
