package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.SwerveSubsystem;
// import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.Vision;
import frc8768.visionlib.helpers.LimelightHelpers;

import java.util.logging.Level;

public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    private boolean isRelocating = false;

    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    public DrivebaseOperator(SwerveSubsystem swerve) {
        super("Drivebase");

        this.swerve = swerve;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);

        // We update odometry on our own thread just in case
        this.swerve.getSwerveDrive().stopOdometryThread();
    }

    @Override
    public void run() {
        swerve.getSwerveDrive().updateOdometry();

        if(controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        if(MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.controllerDeadband) != 0) {
            isRelocating = false;
        }

        if(controller.getAButtonPressed() || isRelocating) {
            isRelocating = true;
            relocate();
        } else {
            // Swerve Example
            swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), false, false, Constants.BOT_CENTER);
        }

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    int failCount = 0;

    /**
     * Relocates the bot based on the nearest AprilTags position, with offsets set in {@link frc8768.robot.util.Constants}
     */
    private void relocate() {
        Vision vision = Robot.getInstance().vision;

        Constants.PoseToTagOffset offset = Constants.PoseToTagOffset.getTagOffsetsForId(vision.getTargetID());
        Object target = vision.getBestTarget();

        if(offset == null || target == null) {
            while(failCount < 1001) {
                offset = Constants.PoseToTagOffset.getTagOffsetsForId(vision.getTargetID());
                target = vision.getBestTarget();

                if(offset == null || target == null) {
                    failCount++;
                } else {
                    break;
                }

                if(failCount == 1000) {
                    failCount = 0;
                    isRelocating = false;
                    LogUtil.LOGGER.log(Level.WARNING, "Relocating failed, no tag with id " + vision.getTargetID() + " and/or no offset assigned.");
                    return;
                }
            }
        }

        Pose3d pose = ((LimelightHelpers.LimelightTarget_Fiducial)target).getRobotPose_TargetSpace();

        double bestScenarioX = pose.getX() - offset.offsetVec.getX();
        double bestScenarioZ = pose.getZ() - offset.offsetVec.getY();

        double bestScenarioR = pose.getRotation().getY();

        if(MathUtil.isNear(offset.offsetVec.getX(), bestScenarioX, 0.0508) &&
                MathUtil.isNear(offset.offsetVec.getY(), bestScenarioZ, 0.0508) &&
                MathUtil.isNear(0, bestScenarioR, 0.0349)) {
            isRelocating = false;
            return;
        }

        Translation2d translation2d = new Translation2d(
                MathUtil.isNear(offset.offsetVec.getY(), bestScenarioZ, 0.0508) ? 0 : MathUtil.clamp(-bestScenarioZ * 0.25d, -0.152, 0.152),
                MathUtil.isNear(offset.offsetVec.getX(), bestScenarioX, 0.0508) ? 0 : MathUtil.clamp(bestScenarioX * 0.25d, -0.152, 0.152));
        swerve.drive(translation2d, MathUtil.isNear(0, bestScenarioR, 0.0349) ? 0 : MathUtil.clamp(bestScenarioR, -0.7, 0.7), false, false, Constants.BOT_CENTER);
    }
}
