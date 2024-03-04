package frc8768.robot.operators;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
// import frc8768.robot.subsystems.TankSubsystemFalcon;
// import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.Vision;
import frc8768.visionlib.helpers.LimelightHelpers;

import java.util.logging.Level;

public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private Command currCommand;


    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    public DrivebaseOperator(SwerveSubsystem swerve, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        super("Drivebase");

        this.swerve = swerve;
        this.arm = armSubsystem;
        this.intake = intakeSubsystem;
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

        // We update odometry on our own thread just in case
        this.swerve.getSwerveDrive().stopOdometryThread();
    }

    @Override
    public void run() {
        this.swerve.getSwerveDrive().updateOdometry();

        if(controller.getBButtonPressed()) {
            this.swerve.getSwerveDrive().zeroGyro();
        }

        if(controller.getXButtonPressed()) {
            swerve.getSwerveDrive().lockPose();
        }

        if(controller.getLeftTriggerAxis() > Constants.controllerDeadband) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
            if(this.arm.getPosition() < 8) {
                this.intake.beginStage(IntakeSubsystem.IntakeStage.INTAKE);
            }
        } else if(controller.getRightBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.LOW);
        } else if(controller.getAButton()) {
            this.intake.beginStage(IntakeSubsystem.IntakeStage.OUTTAKE);
        } else {
            this.arm.releaseLock();
            this.intake.releaseLock();
        }

        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.controllerDeadband));

        boolean joystickInput = MathUtil.applyDeadband(controller.getLeftX(), Constants.controllerDeadband) != 0 || MathUtil.applyDeadband(controller.getLeftY(), Constants.controllerDeadband) != 0 ||
                MathUtil.applyDeadband(controller.getRightX(), Constants.controllerDeadband) != 0;
        boolean isRobotRelative = false;
        if(this.currCommand != null && (joystickInput || controller.getPOV() != -1)) {
            this.currCommand.cancel();
            this.currCommand = null;
        }

        if(controller.getPOV() != -1) {
            isRobotRelative = true;
        }

        if(this.currCommand != null && !this.currCommand.isFinished()) {
            return;
        }

        // Robot-Relative control
        double speed = 0.75;
        switch(controller.getPOV()) {
            case 0 -> translation2d = new Translation2d(speed, 0);
            case 45 -> translation2d = new Translation2d(speed, speed);
            case 90 -> translation2d = new Translation2d(0, speed);
            case 135 -> translation2d = new Translation2d(-speed, speed);
            case 180 -> translation2d = new Translation2d(-speed, 0);
            case 225 -> translation2d = new Translation2d(-speed, -speed);
            case 270 -> translation2d = new Translation2d(0, -speed);
            case 315 -> translation2d = new Translation2d(speed, -speed);
        }
        // Swerve Example
        this.swerve.drive(translation2d, MathUtil.applyDeadband(-controller.getRightX(), Constants.controllerDeadband), !isRobotRelative, false, Constants.BOT_CENTER);

        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

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
