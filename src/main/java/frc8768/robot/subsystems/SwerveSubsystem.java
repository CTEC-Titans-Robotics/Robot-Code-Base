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
     * @param type Neos or Falcons, see {@link MotorType}
     * @throws IOException if it can't find the resources.
     */
    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case NEOS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive(14.5);
            case FALCONS -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive(14.5);
        }
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
        swerveDrive.drive(translation2d, rotation, fieldRelative, isOpenLoop, pivotPoint);
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
}
