package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

public class DrivebaseOperator extends Operator {
    private final SwerveSubsystem swerve;

    // private final ArmSubsystem armSubsystem = new ArmSubsystem(15);
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(16, 17);
    private static final XboxController controller = new XboxController(Constants.driverControllerId);

    public DrivebaseOperator() {
        super("Drivebase");

        swerve = Robot.getInstance().getSwerve();
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);
    }

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
                MathUtil.applyDeadband(Math.pow(-controller.getLeftY(), 3) /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(Math.pow(-controller.getLeftX(), 3) /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        // Swerve Example
        swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, Constants.BOT_CENTER);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
}
