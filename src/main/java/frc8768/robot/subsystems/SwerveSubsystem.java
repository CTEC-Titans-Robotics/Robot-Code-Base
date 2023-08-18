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

    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case NEOS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive();
            case FALCONS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive();
        }
    }

    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean headingCorrection) {
        swerveDrive.drive(translation2d, rotation, fieldRelative, isOpenLoop, headingCorrection);
    }

    public Rotation3d getGyroRot() {
        return swerveDrive.getGyroRotation3d();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
