package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.SwerveSubsystem;
// import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;

import java.util.Map;

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

        if(controller.getXButtonPressed()) {
            swerve.lock();
        }

        if(controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if(controller.getLeftTriggerAxis() > 0.1) {
            swerve.tortoiseMode();
        } else {
            swerve.hareMode();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example

        double speed = .2;
        if(controller.getPOV() == 90) {
            swerve.drive(new Translation2d(0,-speed), 0, false, false, false);
        } else if (controller.getPOV() == 270) {
            swerve.drive(new Translation2d(0,speed), 0, false, false, false);
        } else if (controller.getPOV() == 0) {
            swerve.drive(new Translation2d(speed,0), 0, false, false, false);
        } else if (controller.getPOV() == 180) {
            swerve.drive(new Translation2d(-speed,0), 0, false, false, false);
        } else if (controller.getPOV() == 45) {
            swerve.drive(new Translation2d(speed,-speed), 0, false, false, false);
        } else if (controller.getPOV() == 135) {
            swerve.drive(new Translation2d(-speed,-speed), 0, false, false, false);
        } else if (controller.getPOV() == 225) {
            swerve.drive(new Translation2d(-speed,speed), 0, false, false, false);
        } else if (controller.getPOV() == 315) {
            swerve.drive(new Translation2d(speed,speed), 0, false, false, false);
        } else {
            swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, false);
        }


        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
        for(Map.Entry<String, String> entry : swerve.getDebugInfo().entrySet()) {
            SmartDashboard.putString(entry.getKey(), entry.getValue());
        }
    }
}
