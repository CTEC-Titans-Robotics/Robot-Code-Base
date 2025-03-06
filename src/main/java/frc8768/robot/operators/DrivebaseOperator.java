package frc8768.robot.operators;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

import java.util.List;

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


    double targetInchesX = 0;
    double targetInchesY = 0;
    double targetAngle = 0;

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
        }

        if (controller.getXButton() && controller.getAButton()) {
            elevator.moveDown();
        }

        if (controller.getXButton() && controller.getYButton()) {
            elevator.moveDown();
        }

        if (controller.getLeftBumperButton()) {
            //align(); //TODO left align
        } else if (controller.getRightBumperButton()) {
            //align(); //TODO right align
        }

        //TODO Temporary Target logging. Remove when issue found

        List<LimelightTarget_Fiducial> targets = frontCam.getTargets();
        LimelightTarget_Fiducial target = targets.get(0);
        Pose3d targetPose = target.getTargetPose_RobotSpace();

        if(!targets.isEmpty()) {
            targetInchesX = targetPose.getMeasureX().in(Inches);
            targetInchesY = targetPose.getMeasureY().in(Inches);
            targetAngle = Math.atan(targetInchesY / targetInchesX) * 180 / Math.PI;
        } else {
            targetInchesX = 0;
            targetInchesY = 0;
            targetAngle = 0;
        }

        SmartDashboard.putNumber("Target X", targetInchesX);
        SmartDashboard.putNumber("Target Y", targetInchesY);
        SmartDashboard.putNumber("Target Angle", targetAngle);
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
                robotRelative.getNorm() == 0, true, Constants.BOT_CENTER);




        // Tank Example (Falcons)
        // falconTank.drive(translation2d);

        // Tank Example (Spark)
        // sparkTank.drive(translation2d);
    }
    boolean reangle = false;
    boolean strafe = false;


    public void align() {
        /*
        if(controller.getYButton() && !reangle) {
            reangle = true;
        }

        if(reangle) {
            if (!MathUtil.isNear(0, yaw, 2)) {
                swerve.move(0, 0, MathUtil.clamp(-Math.toRadians(yaw) / 1.5, -0.5, 0.5));
            } else {
                reangle = false;
                swerve.move(0, 0, 0);
            }
        }
*/
        // linear follow attempt;
        if(controller.getRightBumperButton() && !strafe) {
            strafe = true;
        }

        List<LimelightTarget_Fiducial> targets = frontCam.getTargets();
        if(strafe && !targets.isEmpty()) {
            LimelightTarget_Fiducial target = targets.get(0);
            Pose2d targetPose = target.getTargetPose_RobotSpace2D();

            double xMov = 0;
            double yMov = 0;
            double rotMov = 0;

            targetInchesX = targetPose.getMeasureX().in(Inches);
            targetInchesY = targetPose.getMeasureY().in(Inches);
            targetAngle = Math.atan(targetInchesY/targetInchesX)* 180/Math.PI;


            // X: Forward
            if(!MathUtil.isNear(32, targetInchesX, 3)) {
                xMov = -MathUtil.clamp((32-targetInchesX)/60, -0.5, 0.5);
            }

            // Y: Left
            if(!MathUtil.isNear(0, targetInchesX, 3)) {
                yMov = MathUtil.clamp(targetInchesX/50, -0.5, 0.5);
            }


            if(!MathUtil.isNear(0,targetAngle, 12)) {
                rotMov = MathUtil.clamp(-targetAngle/75, -0.1, 0.1);
            }

            if(xMov == 0.0 && yMov == 0.0 && rotMov == 0.0) {
                swerve.move(0, 0, 0);
                strafe = false;
            } else {
                swerve.move(xMov, yMov, rotMov);
            }
        }
    }
}
