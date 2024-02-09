package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.FollowTrajectory;
import frc8768.robot.util.LogUtil;
import frc8768.robot.util.SwerveUtil;
import me.nabdev.pathfinding.structures.ImpossiblePathException;

public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    private Trajectory pathfindingTrajectory;
    private boolean isRelocating = false;

    public DrivebaseOperator(SwerveSubsystem swerve) {
        super("Drivebase");

        this.swerve = swerve;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);

        this.swerve.getSwerveDrive().stopOdometryThread();
    }


    //RUN TIME
    @Override
    public void run() {
        swerve.getSwerveDrive().updateOdometry();

        // intakeSubsystem.tick();

        // if(controller.getLeftBumper()) {
        //     armSubsystem.up();
        // } else if(controller.getLeftTriggerAxis() > 0.1) {
        //     armSubsystem.down();
        // } else {
        //     armSubsystem.stop();
        // }

        // if(controller.getRightBumperPressed()) {
        //     intakeSubsystem.run();
        // }

        if(controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if(controller.getXButtonPressed()) {
            swerve.getSwerveDrive().lockPose();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        if(MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.controllerDeadband) != 0) {
        }

        if(controller.getAButtonPressed() || isRelocating) {
            relocate(new Pose2d(1.90, 7.44, Rotation2d.fromDegrees(90)));
        } else {
            // Swerve Example
            swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, Constants.BOT_CENTER);
        }

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    private void relocate(Pose2d desiredLoc) {
        if(pathfindingTrajectory != null) {
            return;
        }

        try {
            TrajectoryConfig config = new TrajectoryConfig(4.4196 /* Max vel */, 7 /* Max accel */);
            pathfindingTrajectory = Constants.PATHFINDER.generateTrajectory(swerve.getSwerveDrive().getPose(), desiredLoc, config);

            FollowTrajectory command = new FollowTrajectory(pathfindingTrajectory, SwerveUtil.constructHolonomicFromSwerve(swerve), () -> new Rotation2d(90), swerve, swerve);
            command.schedule();
        } catch (ImpossiblePathException e) {
            e.printStackTrace();
        }
    }
}
