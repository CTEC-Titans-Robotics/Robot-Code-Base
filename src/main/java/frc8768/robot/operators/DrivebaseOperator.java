package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;
//import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.helpers.LimelightHelpers.LimelightTarget_Fiducial;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

/**
 * Operator for driving the bot
 */
public class DrivebaseOperator extends Operator {
    private final XboxController controller;
    private final SwerveSubsystem swerve;
   // private final GroundIndefector indefector;
    private final Elevator elevator;
    private final Arm arm;

    private  final LimelightVision frontCam;

    private  final LimelightVision backCam;

    // private final TankSubsystemSpark sparkTank;
    // private final TankSubsystemFalcon falconTank;

    /**
     * Make an instance of the operator
     *
     * @param swerve The required subsystem for this operator.
     */


    //public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, GroundIndefector indefector, Elevator elevator) {
        public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, Elevator elevator, Arm arm, LimelightVision frontCam, LimelightVision backCam) {
        super("Drivebase");

        this.swerve = swerve;
        this.controller = controller;
        this.frontCam = frontCam;
        this.backCam = backCam;

        // sparkTank = Robot.getInstance().getSpark();
        // falconTank = Robot.getInstance().getFalcon();

       // this.indefector = indefector;
        this.elevator = elevator;
            this.arm = arm;

            // Init logging
        LogUtil.registerLogger(swerve::log);
        LogUtil.registerDashLogger(swerve::dashboard);
        LogUtil.registerDashLogger(this::dashboard);
    }

    private Map<String, Object> dashboard() {
        HashMap<String, Object> encoder = new HashMap<>();

        encoder.put("targetX", targetPose.getMeasureX().in(Inches));
        encoder.put("targetY", targetPose.getMeasureY().in(Inches));
        encoder.put("targetRot", targetPose.getRotation().getDegrees());

        return encoder;
    }

    @Override
    public void run() {
        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.CONTROLLER_DEADBAND),
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.CONTROLLER_DEADBAND));

        if (controller.getBButtonPressed()) {
            swerve.getSwerveDrive().zeroGyro();
        }

        if (controller.getYButton()) {
            elevator.moveToState(Elevator.ElevatorState.HANG);
            arm.moveToState(Arm.ArmState.L4);
        }

        if (controller.getAButtonPressed()) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.ZERO);
        }

        if (controller.getXButton() && controller.getAButton()) {
            elevator.moveDown();
        }

        if (controller.getXButton() && controller.getYButton()) {
            elevator.moveDown();
        }

        if (controller.getLeftBumperButton()) {
            align(AlignState.LEFT_ALIGN);
            return;
        } else if (controller.getRightBumperButton()) {
            align(AlignState.RIGHT_ALIGN);
            return;
        }

      /* if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            indefector.spinIntake(true);
        } else if (controller.getLeftBumperButton()) {
            indefector.spinIntake(false);
        } else {
            indefector.stopIntake();
        }

        if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
            // Don't do anything.
        } else if(controller.getRightBumperButton()) {
            indefector.backwards();
        } else if(controller.getRightTriggerAxis() > 0.1) {
            indefector.forward();
        } else {
            indefector.stop();
        }
*/
        double xRobotRelative = 0;
        double yRobotRelative = 0;

        if(controller.getPOV() == 0) {
           if (elevator.state() == Elevator.ElevatorState.L4){
               xRobotRelative = -.05; }
           else
               xRobotRelative = .05;
        } else if (controller.getPOV() == 180) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                xRobotRelative = .05; }
            else
                xRobotRelative = -.05;
        } else if (controller.getPOV() == 90) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                yRobotRelative = -.05; }
            else
                yRobotRelative = .05;
        } else if (controller.getPOV() == 270) {
            if (elevator.state() == Elevator.ElevatorState.L4){
                yRobotRelative = .05; }
            else
                yRobotRelative = -.05;
        }


        double rot = MathUtil.applyDeadband(-controller.getRightX(), Constants.CONTROLLER_DEADBAND);
        if(elevator.state() != Elevator.ElevatorState.ZERO && elevator.state() != Elevator.ElevatorState.L1) {
            translation2d = translation2d.times(0.05);
            rot *= 0.05;
        }

        Translation2d robotRelative = new Translation2d(xRobotRelative, yRobotRelative);

        // Swerve Example
        this.swerve.drive(robotRelative.getNorm() == 0 ? translation2d : robotRelative,
                rot,
                robotRelative.getNorm() == 0, false, Constants.BOT_CENTER);




        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }

    private AlignState currState = AlignState.NONE;
    Pose2d targetPose = Pose2d.kZero;

    private void align(AlignState targetState) {
        List<LimelightTarget_Fiducial> targets = backCam.getTargets();
        System.out.println("189");
        if(!targets.isEmpty()) {
            LimelightTarget_Fiducial target = targets.get(0);
            System.out.println("192");
            if(currState != targetState && target != null) {
                System.out.println("195");
                currState = targetState;
                Pose2d tagToRobotPose = target.getRobotPose_TargetSpace2D();
                targetPose = new Pose2d(new Translation2d(tagToRobotPose.getX() + targetState.x, tagToRobotPose.getY() + targetState.y),
                        tagToRobotPose.getRotation().plus(Rotation2d.fromDegrees(targetState.rotation)));
            }
        }
        if(!targetState.isAtState(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees(), inchesToMeters(0.25), degreesToRadians(2)) && targetPose != Pose2d.kZero) {
            Pose2d drivePose = swerve.getSwerveDrive().getPose();
            swerve.drive(
                    targetPose.getTranslation().minus(drivePose.getTranslation()).times(0.05),
                    (targetPose.getRotation().getDegrees() - drivePose.getRotation().getDegrees()) * 0.05,
                    true,
                    false,
                    Constants.BOT_CENTER
            );
        } else if(currState != AlignState.NONE) {
            currState = AlignState.NONE;
            targetPose = Pose2d.kZero;
            swerve.move(0, 0, 0);
        }
    }

    enum AlignState {

        LEFT_ALIGN(new double[] {6}, 0, inchesToMeters(-7), 0),
        RIGHT_ALIGN(new double[] {6}, 0,inchesToMeters(7),0),
        NONE(new double[] {}, 0, 0, 0);

        final double[] ids;
        final double x;
        final double y;
        final double rotation;

        AlignState(double[] ids, double x, double y, double rotation) {
            this.ids = ids;
            this.x = x;
            this.y = y;
            this.rotation = rotation;
        }

        public boolean isAtState(double x, double y, double rot, double translationTol, double rotTol) {
            return MathUtil.isNear(this.x, x, translationTol) && MathUtil.isNear(this.y, y, translationTol) && MathUtil.isNear(this.rotation, rot, rotTol);
        }
    }
}
