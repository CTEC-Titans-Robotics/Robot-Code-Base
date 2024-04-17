package frc8768.robot.operators;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
//import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    // private final SwerveSubsystem swerve;
    // private final TankSubsystemSpark sparkTank;
    private TankSubsystemFalcon falconTank;
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private Command currCommand;

    public DrivebaseOperator(TankSubsystemFalcon falconTank) {
        super("Drivebase");
        
        // sparkTank = Robot.getInstance().getSpark();
        this.falconTank = falconTank;
    }

    @Override
    public void run() {
        if (falconTank == null) {
            falconTank = Robot.getInstance().getFalcon();
        }
        if (!Robot.isRobotTeleop()) {
            return;
        }
        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(controller.getLeftX() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(controller.getLeftY() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        if ((MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.controllerDeadband) != 0) && currCommand != null) {
            this.currCommand.cancel();
            this.currCommand = null;
        }

        // Tank Example (Falcons)
        falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    /**
     * Relocates the bot based on the nearest AprilTags position, with offsets set in {@link frc8768.robot.util.Constants}
     */
    private void relocate(Pose2d desiredLoc) {
        if(this.currCommand != null && !this.currCommand.isFinished()) {
            return;
        }

        this.currCommand = AutoBuilder.pathfindToPose(desiredLoc, new PathConstraints(
                Units.feetToMeters(Constants.SwerveConfig.MAX_SPEED * Constants.SwerveConfig.MAX_SPEED/10),
                Units.feetToMeters(Constants.SwerveConfig.MAX_SPEED * Constants.SwerveConfig.MAX_SPEED),
                Units.degreesToRadians(450D * 450D), Units.degreesToRadians(720D * 720D/10D)));
        this.currCommand.schedule();
    }
}
