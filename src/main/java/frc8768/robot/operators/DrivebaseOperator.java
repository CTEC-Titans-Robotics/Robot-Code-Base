package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.SwerveSubsystem;
// import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;

public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    public DrivebaseOperator() {
        super("Drivebase");

        swerve = Robot.getInstance().getSwerve();
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();
    }

    @Override
    public void run() {
        swerve.getSwerveDrive().updateOdometry();

        if(controller.getBButton()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if(controller.getLeftTriggerAxis() > 0.1 && swerve.isTortoise()) {
            swerve.hareMode();
        } else if(controller.getLeftTriggerAxis() > 0.1 && !swerve.isTortoise()) {
            swerve.tortoiseMode();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example
        swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, false);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
