package frc8768.robot.operators;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;
//import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.helpers.LimelightHelpers;
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
    /**
     * OTT Vision
     */
    private boolean autoAlignActive = false;
    private double snapshotX = 0;
    private double snapshotY = 0;
    private double snapshotRot = 0;
    private double driveStartTime = 0;
    private final double driveTimeout = 3.0;

    private final PIDController xPID = new PIDController(1.5, 0, 0);
    private final PIDController yPID = new PIDController(1.5, 0, 0);
    private final PIDController rotPID = new PIDController(0.05, 0, 0);

    //END OTT Vision




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
///        public DrivebaseOperator(XboxController controller, SwerveSubsystem swerve, LimelightVision frontCam, LimelightVision backCam) {
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
        return new HashMap<>();
    }
    //Controller Overall Speed
    public double speedscale = 0.50;  //Value between 0 and 1, Drive Sticks
    public double turtlespeedscale = 0.1;  //Value between 0 and 1, Driver Left Trigger
    public double slowspeedscale = 0.4;  //Value between 0 and 1, Driver Right Trigger
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    private boolean isValidPose(double[] pose) {
        return Math.abs(pose[0]) < 5 && Math.abs(pose[1]) < 5 && Math.abs(pose[5]) < 180;
    }

    Command currCommand = Constants.DEFAULT_COMMAND;
    @Override
    public void run() {


        // Apply controller deadband
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY() /* For Tank, use controller.getLeftY() */, Constants.CONTROLLER_DEADBAND)*speedscale,
                MathUtil.applyDeadband(-controller.getLeftX() /* For Tank, use controller.getRightY() */, Constants.CONTROLLER_DEADBAND)*speedscale
        );

        if (controller.getStartButtonPressed()) {
            swerve.zeroGyro();
        }

        if (controller.getAButtonPressed()) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.L2);
        }

        if(currCommand != Constants.DEFAULT_COMMAND) {
            if(currCommand.isFinished()) {
                currCommand = Constants.DEFAULT_COMMAND;
            }

            if(translation2d.getNorm() != 0) {
                currCommand.cancel();
                currCommand = Constants.DEFAULT_COMMAND;
                return;
            }
        } else {
            if(controller.getLeftBumperButtonPressed()) {
                align(AlignState.LEFT_ALIGN);
            } else if(controller.getRightBumperButtonPressed()) {
                align(AlignState.RIGHT_ALIGN);
            }
        }

        /**
         * OTT Vision
         */


/*
        if (controller.getLeftBumperButton() && !autoAlignActive) {
            double[] pose = NetworkTableInstance.getDefault()
                    .getTable("limelight-back")
                    .getEntry("camerapose_targetspace")
                    .getDoubleArray(new double[6]);
//These are the correct components of the array
            if (pose.length >= 6 && isValidPose(pose)) {
                snapshotX = pose[2]; // Forward distance
                snapshotY = pose[0];// + 0.1778; // 7 inches left of tag
                snapshotRot = pose[4]; // Tag's rotation

                driveStartTime = Timer.getFPGATimestamp();
                autoAlignActive = true;

                System.out.println("Auto-align started → X: " + snapshotX + " Y: " + snapshotY + " Rot: " + snapshotRot);
            } else {
                System.out.println("No tag visible on backVision — can't auto-align.");
            }
        }
        // === STEP 2: Execute drive to target ===
        if (autoAlignActive) {
            double elapsed = Timer.getFPGATimestamp() - driveStartTime;

            Pose2d currentPose = swerve.getSwerveDrive().getPose(); // Or your equivalent odometry call
            Pose2d goalPose = currentPose.relativeTo(
                    new Pose2d(new Translation2d(snapshotX, snapshotY), Rotation2d.fromDegrees(snapshotRot))
            );

            double xSpeed = clamp(xPID.calculate(currentPose.getX(), goalPose.getX()), -1.0, 1.0);
            double ySpeed = clamp(yPID.calculate(currentPose.getY(), goalPose.getY()), -1.0, 1.0);
            double rotSpeed = clamp(rotPID.calculate(currentPose.getRotation().getDegrees(), snapshotRot), -0.5, 0.5);

            swerve.move(xSpeed, ySpeed, rotSpeed); // robot-relative

            if (elapsed > driveTimeout) {
                swerve.move(0, 0, 0);
                autoAlignActive = false;
                System.out.println("Auto-align complete or timed out.");
            }
        }
 */

        //END OTT Vision

        if (controller.getXButton()){
            if(arm.getRollersCurrent() > 18) {
                swerve.setTargetHeading(translation2d, 120);
            } else {swerve.setTargetHeading(translation2d, 128);}
///            swerve.setTargetHeading(translation2d, 128);
            return;
        } else if (controller.getBButton()){
            if(arm.getRollersCurrent() > 18) {
                swerve.setTargetHeading(translation2d, -120);
            } else {swerve.setTargetHeading(translation2d, -128);}
///                swerve.setTargetHeading(translation2d, -128);
            return;
        } else if(controller.getXButtonReleased() || controller.getBButtonReleased()) {
            swerve.setTargetHeading(translation2d, 0);
        }
/*
        if (controller.getXButton() && controller.getAButton()) {
            elevator.moveDown();
        }

        if (controller.getXButton() && controller.getYButton()) {
            elevator.moveDown();
        }
*/
/* remove Align
        if (controller.getLeftBumperButton()) {
            align(AlignState.LEFT_ALIGN);
            return;
        } else if (controller.getRightBumperButton()) {
            align(AlignState.RIGHT_ALIGN);
            return;
        } else if(controller.getYButtonPressed()) {
            align(AlignState.CENTER);
        }
*/
/*  Removed Algae
       if(controller.getRightBumperButton() && controller.getRightTriggerAxis() > 0.1) {
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
        //Speed Scaling
        double rot = MathUtil.applyDeadband(-controller.getRightX(), Constants.CONTROLLER_DEADBAND);
        //Turtle Mode
        if(controller.getLeftTriggerAxis() > 0.1){
            translation2d = translation2d.times(turtlespeedscale);
            rot *= turtlespeedscale;
        }
        //Slower Speed
        if(controller.getRightTriggerAxis() > 0.1){
            translation2d = translation2d.times(slowspeedscale);
            rot *= slowspeedscale;
        }

        //Robot Relative MOvement
        double xRobotRelative = 0;
        double yRobotRelative = 0;
        if(controller.getPOV() == 0) {
           if (elevator.state() == Elevator.ElevatorState.ZERO) {
               xRobotRelative = -.05; }
           else
               xRobotRelative = .05;
        } else if (controller.getPOV() == 180) {
            if (elevator.state() == Elevator.ElevatorState.ZERO) {
                xRobotRelative = .05; }
            else
                xRobotRelative = -.05;
        } else if (controller.getPOV() == 90) {
            if (elevator.state() == Elevator.ElevatorState.ZERO) {
                yRobotRelative = -.05; }
            else
                yRobotRelative = .05;
        } else if (controller.getPOV() == 270) {
            if (elevator.state() == Elevator.ElevatorState.ZERO) {
                yRobotRelative = .05; }
            else
                yRobotRelative = -.05;
        }

        //Slower speed when elevator is up
        if(elevator.state() == Elevator.ElevatorState.L3 || elevator.state() == Elevator.ElevatorState.L4) {
            translation2d = translation2d.times(0.2);
            rot *= 0.2;
        }

        Translation2d robotRelative = new Translation2d(xRobotRelative, yRobotRelative);

        // Swerve Example
        this.swerve.drive(robotRelative.getNorm() == 0 ? translation2d : robotRelative,
                rot,
                robotRelative.getNorm() == 0, false, Constants.BOT_CENTER);
    }


    private void align(AlignState targetState) {
        /*
        if(currState != targetState) {
            currState = targetState;
            swerve.getSwerveDrive().resetOdometry(new Pose2d(Translation2d.kZero, swerve.getSwerveDrive().getYaw()));
        }
        List<LimelightTarget_Fiducial> targets = backCam.getTargets();

        boolean atState = false;
        if(!targets.isEmpty() && targetState == AlignState.CENTER) {
            targetPose = targets.get(0).getRobotPose_TargetSpace2D();
        }

        if(targetState != AlignState.CENTER && targetState != AlignState.NONE) {
            Translation2d desired = new Translation2d(targetState.x, targetState.y);
            targetPose = new Pose2d(swerve.getSwerveDrive().getPose().getTranslation().plus(desired),
                    Rotation2d.fromDegrees(swerve.getSwerveDrive().getYaw().getDegrees() + targetState.rotation));
        }

        Pose2d drivePose = swerve.getSwerveDrive().getPose();
        if(targetPose != Pose2d.kZero) {
            Transform2d transform = targetPose.minus(drivePose);
            if (MathUtil.isNear(0, transform.getX(), 0.25) && MathUtil.isNear(0, transform.getY(), 0.25) && MathUtil.isNear(0, transform.getRotation().getDegrees(), 2)) {
                atState = true;
            } else {
                swerve.drive(transform.getTranslation().times(0.05), transform.getRotation().getDegrees(), true, false, Constants.BOT_CENTER);
            }
        }

        if(currState != AlignState.NONE && atState) {
            currState = AlignState.NONE;
            targetPose = Pose2d.kZero;
        }
         */

        Constants.DesiredPoses desiredPose = Constants.DesiredPoses.getClosest(swerve.getSwerveDrive().getPose());

        Translation2d offset = new Translation2d(targetState.x, targetState.y).rotateBy(desiredPose.getDesiredPose().getRotation());
        Pose2d offsetPose = new Pose2d(desiredPose.getDesiredPose().getTranslation().plus(offset), desiredPose.getDesiredPose().getRotation());

        currCommand = AutoBuilder.pathfindToPose(offsetPose, Constants.DEFAULT_CONSTRAINTS);
        currCommand.schedule();
    }

    enum AlignState {
        LEFT_ALIGN(0, inchesToMeters(-7), Rotation2d.kZero),//tag 6, 0 front/back, 7 left, 0 rotation
        RIGHT_ALIGN(0, inchesToMeters(7), Rotation2d.kZero),//tag 6, 0 front/back, 7 right, 0 rotation
        CENTER(inchesToMeters(7), 0, Rotation2d.kZero),
        NONE(0, 0, Rotation2d.kZero);

        final double x;
        final double y;
        final Rotation2d rotation;

        AlignState(double x, double y, Rotation2d rotation) {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
        }

        public boolean isAtState(double x, double y, Rotation2d rot, double translationTol, double rotTol) {
            return MathUtil.isNear(this.x, x, translationTol) && MathUtil.isNear(this.y, y, translationTol) && MathUtil.isNear(this.rotation.getDegrees(), rot.getDegrees(), rotTol);
        }
    }
}