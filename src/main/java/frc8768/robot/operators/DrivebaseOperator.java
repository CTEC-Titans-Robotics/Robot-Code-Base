package frc8768.robot.operators;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.ClimberSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.driverControllerId);
    private final SwerveSubsystem swerve;
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private Command currCommand;


    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    /**
     * @param swerve The required subsystem for this operator.
     */
    public DrivebaseOperator(SwerveSubsystem swerve, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        super("Drivebase");

        this.swerve = swerve;
        this.arm = armSubsystem;
        this.intake = intakeSubsystem;
        this.climber = new ClimberSubsystem();
        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();
    }

    public void initTeleop() {
        this.climber.init();
    }

    @Override
    public void run() {
        if(controller.getBButtonPressed()) {
            this.swerve.getSwerveDrive().zeroGyro();
        }

        if(controller.getAButton()) {
            this.climber.down();
        } else if(controller.getYButton()) {
            this.climber.up();
        } else {
            this.climber.stop();
        }

        if(controller.getLeftTriggerAxis() > Constants.controllerDeadband) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
            if(this.arm.getPosition() < 8) {
                this.intake.beginStage(IntakeSubsystem.IntakeStage.INTAKE);
            }
        } else if(controller.getXButton()) {
            this.intake.beginStage(IntakeSubsystem.IntakeStage.OUTTAKE);
        } else if(controller.getRightBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.LOW);
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

        // if(controller.getLeftBumperPressed()) {
        //     this.relocate(Constants.FieldWaypoints.AMP.getPose2d());
        // }

        // if(controller.getRightTriggerAxis() > Constants.controllerDeadband) {
        //     visionMeasureOdometry();
        // }

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

    private void visionMeasureOdometry() {
        PhotonPipelineResult target = Robot.getInstance().getLeftVision().getBestTarget();
        if(target == null)
            return;
        if(!target.hasTargets())
            return;

        Pose3d pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestTarget().getBestCameraToTarget(),
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)
                        .getTagPose(target.getBestTarget().getFiducialId()).get(),
                new Transform3d(Units.inchesToMeters(5.25), Units.inchesToMeters(10.75), -Units.inchesToMeters(19.25),
                        new Rotation3d(Units.degreesToRadians(-30), Units.degreesToRadians(0), Units.degreesToRadians(0))));
        this.swerve.getSwerveDrive().addVisionMeasurement(pose.toPose2d(), target.getTimestampSeconds());
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
                Units.degreesToRadians(225 * 225), Units.degreesToRadians(450D * 450D/10D)));
        this.currCommand.schedule();
    }
}
