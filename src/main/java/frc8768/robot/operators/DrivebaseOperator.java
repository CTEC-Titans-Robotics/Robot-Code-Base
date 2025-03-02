package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.DRIVER_CONTROLLER_ID);
    private final SwerveSubsystem swerve;
    private Command currCommand;


    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    /**
     * Make an instance of the operator
     *
     * @param swerve The required subsystem for this operator.
     */
    public DrivebaseOperator(SwerveSubsystem swerve) {
        super("Drivebase");

        this.swerve = swerve;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);
    }

    @Override
    public void run() {
        if(controller.getBButtonPressed()) {
            this.swerve.getSwerveDrive().zeroGyro();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.CONTROLLER_DEADBAND));

        if((MathUtil.applyDeadband(controller.getLeftX(), Constants.CONTROLLER_DEADBAND) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.CONTROLLER_DEADBAND) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.CONTROLLER_DEADBAND) != 0) && currCommand != null) {
            this.currCommand.cancel();
            this.currCommand = null;
        }

        if(this.currCommand != null && !this.currCommand.isFinished()) {
            return;
        }
        // Swerve Example
        this.swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.CONTROLLER_DEADBAND), true, true, Constants.BOT_CENTER);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
