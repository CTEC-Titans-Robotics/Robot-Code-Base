package frc8768.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.MotorType;
import swervelib.SwerveDrive;
import swervelib.imu.Pigeon2Swerve;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.io.IOException;

/*
    Subsystems are NOT threaded to avoid a race condition with their operators.
 */
public class SwerveSubsystem implements Subsystem {
    private SwerveDrive swerveDrive;
    private boolean isTortoise;

    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case NEOS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive();
            case FALCONS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive();
        }
    }

    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean headingCorrection) {
        swerveDrive.drive(translation2d.times(
                isTortoise ? 1.0625 : swerveDrive.swerveDriveConfiguration.maxSpeed),
                isTortoise ? (rotation * (((90*5) * Math.PI)/180)) : (rotation * (((360*5) * Math.PI)/180)) ,
                fieldRelative, isOpenLoop, headingCorrection);
    }

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

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
