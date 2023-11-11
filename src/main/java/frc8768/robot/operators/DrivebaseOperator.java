package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;

public class DrivebaseOperator extends Operator {
    private final SwerveSubsystem swerve;

    private final ArmSubsystem armSubsystem = new ArmSubsystem(15);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(16, 17);
    private static final XboxController controller = new XboxController(Constants.driverControllerId);

    public DrivebaseOperator() {
        super("Drivebase");

        swerve = Robot.getInstance().getSwerve();
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();
    }

    @Override
    public void run() {
        swerve.getSwerveDrive().updateOdometry();
        intakeSubsystem.tick();
        armSubsystem.tick();

        if(controller.getLeftBumper()) {
            armSubsystem.up();
        } else if(controller.getLeftTriggerAxis() > 0.1) {
            armSubsystem.down();
        } else {
            armSubsystem.stop();
        }

        if(controller.getRightBumperPressed()) {
            intakeSubsystem.run();
        }

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

        // Swerve Example
        swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), true, false, false);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    public ArmSubsystem getArmSubsystem() {
        return armSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }
}
