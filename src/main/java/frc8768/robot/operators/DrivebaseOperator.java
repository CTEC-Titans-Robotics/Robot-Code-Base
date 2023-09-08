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
    private TankSubsystemFalcon falconTank;
    private static final XboxController controller = new XboxController(Constants.driverControllerId);

    public DrivebaseOperator() {
        super("Drivebase");
    }

    @Override
    public void run() {
        if(falconTank == null) {
            falconTank = Robot.getInstance().getFalcon();
        }
        if (!Robot.isRobotTeleop()) {
            return;
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getRightY() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example
        // swerve.drive(translation2d, -controller.getRightX(), true, false, true);

        // Tank Example (Falcons)
        falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
