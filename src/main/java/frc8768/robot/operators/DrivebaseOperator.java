package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
//import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;

public class DrivebaseOperator extends Operator {
    // private final SwerveSubsystem swerve;
    // private final TankSubsystemSpark sparkTank;
    private final TankSubsystemFalcon falconTank;

    public DrivebaseOperator() {
        super("Drivebase", new XboxController(Constants.driverControllerId));

        // swerve = Robot.getInstance().getSwerve();
        // sparkTank = Robot.getInstance().getSpark();
        falconTank = Robot.getInstance().getFalcon();
        init();
    }

    public XboxController getHid() {
        return (XboxController) hid;
    }

    @Override
    public void run() {
        if (!isRobotTeleop()) {
            return;
        }
        XboxController controller = this.getHid();

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(controller.getRightY() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example
        // swerve.drive(translation2d, -controller.getRightX(), true, false, true);

        // Tank Example (Falcons)
        falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
