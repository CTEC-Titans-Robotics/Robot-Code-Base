// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.PhotonVision;
import frc8768.visionlib.Vision;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

//PhotonVision
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.List;
import java.util.logging.Level;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    public SparkMax sparkMax;
    XboxController controller = new XboxController(0);

    /**
     * Robot instance, can't be seen across threads
     */
    public static Robot instance;

    /**
     * Drivebase Operator
     */
    private DrivebaseOperator drivebase;

    /**
     * The swerve subsystem, held in here for Auton.
     */
    private SwerveSubsystem swerve;

    /**
     * Vision API instance
     */
    public Vision vision;

    PigeonIMU pigeon = new PigeonIMU(20);
    /**
     * Auton Instance
     */
    private Auto auto;

    public Robot() {
        instance = this;
    }

    /**
     * Certain properties cannot be seen across Threads.
     *
     * @return The Robot instance;
     */
    public static Robot getInstance() {
        return instance;
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //CameraServer.startAutomaticCapture();
        sparkMax = new SparkMax(15, MotorType.kBrushless);
        try {
          swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }


        this.drivebase = new DrivebaseOperator(this.swerve);
        //this.auto = new Auto(swerve);
        this.vision = new PhotonVision(PhotonVision.Type.LIMELIGHT);

        this.drivebase.init();
    }

    public Vision getVision() {
        return this.vision;
    }

    /**
     * Runs even if the Robot is disabled.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if(this.drivebase != null && !this.drivebase.isAlive()) {
            LogUtil.LOGGER.log(Level.WARNING, "Drivebase thread died! Reviving...");
            this.drivebase.reviveThread();
        }

        LogUtil.run();
    }

    /**
     * Runs when first entering Autonomous mode
     */
    @Override
    public void autonomousInit() {
/*        if (this.auto != null) {
            this.auto.getSelected().schedule();
       }
        swerve.getSwerveDrive().resetOdometry(new Pose2d());
*/
    }

    // Target distance in meters (1 foot = 0.3048 meters)
    @Override
    public void autonomousPeriodic() {
//        auto.Translation(0.3048,0.3048/2);
    }

    /**
     * Runs at the start of Teleop state
     */
    @Override
    public void teleopInit() {}

    /**
     * Runs every "tick" of Teleop time
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Runs at the start of Test state
     */
    @Override
    public void testInit() {
        swerve.getSwerveDrive().resetOdometry(new Pose2d());
        relocate = false;
        reangle = false;
        reposition = false;
        strafe = false;
    }

    private void move(double xSpeed, double ySpeed, double rot) {
        swerve.drive(new Translation2d(xSpeed, ySpeed), rot, false, true, Constants.BOT_CENTER);
    }


    boolean relocate = false; //move forward 1 ft
    boolean reangle = false; //rotate towards april tag

    boolean reposition = false; //move towards april tag
    boolean strafe = false; // Move left or right to center april tag
    /**
     * Runs every "tick" of Test time
     */
    @Override
    public void testPeriodic() {

        for (swervelib.SwerveModule module : this.swerve.getSwerveDrive().getModules()) {
            SmartDashboard.putNumber("Module" + module.moduleNumber + " Encoder", module.getAbsolutePosition());
        }

        // Display vision data on SmartDashboard
        var table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("Left");
        boolean hasTarget = table.getEntry("hasTarget").getBoolean(false);
        double yaw = table.getEntry("targetYaw").getDouble(0.0);
        //double yaw = vision.getTargetYaw();
        double distY = table.getEntry("targetPixelsY").getDouble(0.0);

        SmartDashboard.putBoolean("Has Target", hasTarget);
        SmartDashboard.putNumber("Target Yaw", yaw);
        //Motor movement

        if(controller.getXButton() && !relocate) {
            relocate = true;
        }
        double targetX = -0.304;

        if(relocate) {
            Pose2d pose = swerve.getSwerveDrive().getPose(); // Odometry
            if(!MathUtil.isNear(targetX, pose.getX(), 0.001)) {
                move(MathUtil.clamp((targetX-pose.getX())*2, -0.1, 0.1), 0,0);
            } else {
                relocate = false;
                move(0, 0,0);
            }
        }

        if(controller.getBButton() && !reposition) {
            reposition = true;
        }
        double distX = vision.getDistanceToTarget(0,15.75,24.5,false);
        if(reposition && distX != -1) {
            if(!MathUtil.isNear(12, distX, 0.001)) {
                move(MathUtil.clamp(12-distX*2, -0.1, 0.1), 0,0);
            } else {
                reposition = false;
                move(0, 0,0);
            }
        }
/*
        class relocate {
            private void move(double xSpeed, double ySpeed, double xDist, double yDist) {
                // Add your sensor checking logic
                swerve.drive(new Translation2d(xSpeed,ySpeed),0,false,true,new Translation2d(0,0));


            }
        }

            // Instantiate the local class and use it
        relocate moveBot = new relocate();

        if (controller.getXButton()) {
            moveBot.move(0.1,0,0.304,0);

            //            swerve.drive(new Translation2d(.1,0),0,false,true,new Translation2d(0,0));
        }

        if (controller.getYButton()) {
            moveBot.move(0,0.1,0,0.304);

            //            swerve.drive(new Translation2d(.1,0),0,false,true,new Translation2d(0,0));
        }


        Translation2d targetPoint1 = new Translation2d(0.304,0); // Target position in meters
        Translation2d currentPosition1 = swerve.getSwerveDrive().getPose().getTranslation();
        double distanceToTarget1 = currentPosition1.getDistance(targetPoint1);
        SmartDashboard.putNumber("D2T1", distanceToTarget1);
        if(distanceToTarget1 < 0.05){
            swerve.drive(new Translation2d(0, 0), 0, false, false,new Translation2d(0,0));
            swerve.getSwerveDrive().resetOdometry(new Pose2d());
        }

        Translation2d targetPoint2 = new Translation2d(0,0.304); // Target position in meters
        Translation2d currentPosition2 = swerve.getSwerveDrive().getPose().getTranslation();
        double distanceToTarget2 = currentPosition2.getDistance(targetPoint2);
        SmartDashboard.putNumber("D2T2", distanceToTarget2);
        if(distanceToTarget2 < 0.05){
            swerve.drive(new Translation2d(0, 0), 0, false, false,new Translation2d(0,0));
            //swerve.getSwerveDrive().resetOdometry(new Pose2d());
        }

*/

        /*f (controller.getAButton()) {
            swerve.drive(new Translation2d(0, 0), 0, false, false,new Translation2d(0,0));
            swerve.getSwerveDrive().resetOdometry(new Pose2d());
            if (hasTarget && yaw > 5) {
                swerve.drive(new Translation2d(0, 0), 0.1, false, true, Constants.BOT_CENTER);
            } else if (hasTarget && yaw < -5) {
                swerve.drive(new Translation2d(0, 0), -0.1, false, true, Constants.BOT_CENTER);
                //sparkMax.set(-0.1);
            } else if (hasTarget) {
                sparkMax.set(0);
            } else {
                sparkMax.set(0);
            }
        }
*/
        if(controller.getYButton() && !reangle) {
            reangle = true;
        }

        if(reangle) {
            if (!MathUtil.isNear(0, yaw, 2)) {
                move(0, 0, MathUtil.clamp(-Math.toRadians(yaw) / 1.5, -0.5, 0.5));
            } else {
                reangle = false;
                move(0, 0, 0);
            }
        }

        // linear follow attempt;
        if(controller.getXButton() && !strafe) {
            strafe = true;
        }

        List<?> targets = vision.getTargets();
        if(strafe && !targets.isEmpty()) {
            PhotonTrackedTarget target = (PhotonTrackedTarget) targets.get(0);
            Transform3d transform = target.getBestCameraToTarget();
            double xMov = 0;
            double yMov = 0;
            double rotMov = 0;

            // X: Forward
            if(!MathUtil.isNear(32, transform.getMeasureX().in(Inches), 2)) {
                xMov = -MathUtil.clamp((32-transform.getMeasureX().in(Inches))*4, -0.2, 0.2);
            }

            // Y: Left
            double yTranslation = transform.getMeasureY().in(Inches);
            if(!MathUtil.isNear(0, yTranslation, 1)) {
                yMov = MathUtil.clamp(yTranslation, -0.2, 0.2);
            }

            if(!MathUtil.isNear(-180, transform.getRotation().getMeasureZ().in(Degree), 3)) {
                rotMov = MathUtil.clamp(-transform.getRotation().getMeasureZ().in(Degree), -0.1, 0.1);
            }

            if(xMov == 0.0 && yMov == 0.0 && rotMov == 0.0) {
                move(0, 0, 0);
            } else {
                move(xMov, yMov, rotMov);
            }
        }

      /*  if (controller.getAButton()) {
            swerve.drive(new Translation2d(0, 0), 0, false, false,new Translation2d(0,0));
            swerve.getSwerveDrive().resetOdometry(new Pose2d());
            if (hasTarget && yaw > 5) {
                sparkMax.set(0.1);
                swerve.drive(Rotation2d(yaw));
            } else if (hasTarget && yaw < -5) {
                sparkMax.set(-0.1);
            } else if (hasTarget) {
                sparkMax.set(0);
            } else {
                sparkMax.set(0);
            }
        }
        */


/*if (controller.getYButton()) {
    swerve.drive(new Translation2d(0 ,0), 0.25, false, true, Constants.BOT_CENTER);

} else if (controller.getLeftTriggerAxis() > 0.1){
    swerve.drive(new Translation2d(0 ,0), -0.25, false, true, Constants.BOT_CENTER);
} else {
    swerve.drive(new Translation2d(0 ,0), 0.0, false, true, Constants.BOT_CENTER);
}
*/
/*
        Translation2d translation2d = new Translation2d(
                MathUtil.applyDeadband(-controller.getLeftY(), Constants.controllerDeadband),
                MathUtil.applyDeadband(-controller.getLeftX(), Constants.controllerDeadband));

double speed = 0.25;
switch(controller.getPOV()) {
            case 0 -> sparkMax.set(-0.1);//translation2d = new Translation2d(speed,0);
            //case 45 -> translation2d = new Translation2d(speed, speed);
            case 90 -> sparkMax.set(0.1);//translation2d = new Translation2d(0, speed);
            //case 135 -> translation2d = new Translation2d(-speed, speed);
            case 180 -> sparkMax.set(-0.2);//translation2d = new Translation2d(-speed,0);
            //case 225 -> translation2d = new Translation2d(-speed, -speed);
            case 270 -> sparkMax.set(0.2);//translation2d = new Translation2d(0, -speed);
            //case 315 -> translation2d = new Translation2d(speed, -speed);
            case -1 -> sparkMax.set(0);//translation2d = new Translation2d(0, -speed);
            //case 315 -> translation2d = new Translation2d(speed, -speed);
        }
 */
    }
}
