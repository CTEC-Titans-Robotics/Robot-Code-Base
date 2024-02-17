package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.Robot;
import frc8768.robot.util.MathUtil;
import frc8768.robot.util.MotorType;
import frc8768.robot.util.Constants;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.Vision;
import frc8768.visionlib.helpers.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
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
    private VisionOdomThread visionUpdateThread;

    /**
     * @param type Neos or Falcons, see {@link MotorType}
     * @throws IOException if it can't find the resources.
     */
    public SwerveSubsystem(MotorType type) throws IOException {
        switch(type) {
            case SPARKMAX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/neo")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED);
            case TALONFX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/falcon")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED);
            case SPARKFLEX -> swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve/sparkflex")).createSwerveDrive(Constants.SwerveConfig.MAX_SPEED);
        }

        this.visionUpdateThread = new VisionOdomThread(this, Robot.getInstance().getLimelightVision(), "VisionOdom Thread");
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
            SwerveModule module = swerveDrive.getModules()[i];

            map.put(String.format("Module %d Drive: Velocity", i),
                    String.valueOf(module.getDriveMotor().getVelocity()));
            map.put(String.format("Module %d Angle: Velocity", i),
                    String.valueOf(module.getAngleMotor().getVelocity()));

            map.put(String.format("Module %d Drive: Position", i),
                    String.valueOf(module.getDriveMotor().getPosition()));
            map.put(String.format("Module %d Angle: Position", i),
                    String.valueOf(module.getAngleMotor().getPosition()));

            Object angleMotorObj = module.getAngleMotor().getMotor();
            Object driveMotorObj = module.getDriveMotor().getMotor();
            if(angleMotorObj instanceof CANSparkMax sparkMax) {
                map.put(String.format("Module %d Angle: Temp", i),
                        String.valueOf(sparkMax.getMotorTemperature()));
            }
            if(driveMotorObj instanceof CANSparkMax sparkMax) {
                map.put(String.format("Module %d Drive: Temp", i),
                        String.valueOf(sparkMax.getMotorTemperature()));
            }
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
        private LimelightVision vision;

        public VisionOdomThread(SwerveSubsystem swerve, LimelightVision vision, String name) {
            super(name);
            this.swerve = swerve;
            this.vision = vision;
        }

        @Override
        public void run() {
            int i = 0;
            while(true) {
                i++;
                if(i < 50000) {
                    continue;
                } else if(i > 50000) {
                    i = 0;
                }
                LimelightHelpers.Results target = (LimelightHelpers.Results) vision.getBestTarget();
                if(target.targets_Fiducials.length == 0)
                    continue;

                Pose2d pose = new Pose2d(target.getBotPose2d_wpiBlue().getTranslation(), new Rotation2d(0, target.targets_Fiducials[0].getRobotPose_FieldSpace().getRotation().getZ()));
                this.swerve.getSwerveDrive().addVisionMeasurement(pose, Timer.getFPGATimestamp());
                if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    this.swerve.getSwerveDrive().setGyro(new Rotation3d(0, 180, 0));
                }
            }
        }
    }
}
