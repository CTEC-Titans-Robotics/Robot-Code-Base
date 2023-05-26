package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;

import java.io.IOException;

public class DrivebaseOperator extends Operator {
    private final SwerveSubsystem swerve;

    public DrivebaseOperator() {
        super("Drivebase", new XboxController(Constants.driverControllerId));

        swerve = Robot.getInstance().getSwerve();

        init();
    }

    public XboxController getHid() {
        return (XboxController) hid;
    }

    @Override
    public void run() {
        while(true) {
            if (!isRobotTeleop()) {
                continue;
            }
            XboxController controller = this.getHid();
            Translation2d translation2d = new Translation2d(
                    MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband),
                    MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband));

            swerve.drive(translation2d, -controller.getRightX(), true, false, true);
        }
    }
}
