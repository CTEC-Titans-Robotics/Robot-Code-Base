// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.AuxiliaryOperator;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;
import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.Vision;

import frc8768.visionlib.multicam.PhotonMultiCam;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.io.IOException;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{

    private static final XboxController driveController = new XboxController(Constants.DRIVER_CONTROLLER_ID);
    private static final XboxController auxController = new XboxController(1);

    /**
     * Robot instance, can't be seen across threads
     */
    public static Robot instance;

    /**
     * Drivebase Operator
     */
    private DrivebaseOperator drivebase;
    private AuxiliaryOperator auxiliary;

    /**
     * The swerve subsystem, held in here for Auton.
     */


    private SwerveSubsystem swerve;
    private GroundIndefector groundIndefector;
    private Elevator elevator;
    private Arm arm;
    // private TankSubsystemFalcon falcon;
    // private TankSubsystemSpark spark;

    /**
     * Vision API instance
     */
    public LimelightVision frontVision, backVision;
    public PhotonMultiCam robotCams;

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
        CameraServer.startAutomaticCapture();
        frontVision = new LimelightVision("limelight-front");
        backVision = new LimelightVision("limelight-back");

        /*  ////Add back for Photonvision on Swerve
        robotCams = new PhotonMultiCam();
        robotCams.addCamera("fl", new Transform3d(-0.301516, 0.301516, 0.184,
                new Rotation3d(Math.toRadians(10),Math.toRadians(-45),Math.toRadians(0))));
        robotCams.addCamera("fr", new Transform3d(0.301516, 0.301516, 0.184,
                new Rotation3d(Math.toRadians(10),Math.toRadians(45),Math.toRadians(0))));
        robotCams.addCamera("bl", new Transform3d(-0.301516, -0.301516, 0.184,
                new Rotation3d(Math.toRadians(10),Math.toRadians(-135),Math.toRadians(0))));
        robotCams.addCamera("br", new Transform3d(0.301516, -0.301516, 0.184,
                new Rotation3d(Math.toRadians(10),Math.toRadians(135),Math.toRadians(0))));
        */

        try {
          this.swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }

        // this.groundIndefector = new GroundIndefector();
        this.elevator = new Elevator();
        this.arm = new Arm();

        //Pass systems to Operators
        this.drivebase = new DrivebaseOperator(driveController, this.swerve, this.elevator, this.arm, this.frontVision, this.backVision);
        this.auxiliary = new AuxiliaryOperator(auxController, this.elevator, this.arm);
        this.auto = new Auto(swerve, arm, elevator);
        ////Not Limelight this way right now, using Pathplanner
        // this.vision = new LimelightVision("limelight");

        this.auxiliary.init();
        this.drivebase.init();

        CommandScheduler.getInstance().registerSubsystem(swerve);
    }

    /**
     * Runs even if the Robot is disabled.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        LogUtil.run();
        elevator.tick();
        arm.tick();
        SmartDashboard.putNumber("Swerve X", swerve.getSwerveDrive().getPose().getX());
        SmartDashboard.putNumber("Swerve Y", swerve.getSwerveDrive().getPose().getY());
        SmartDashboard.putString("Swerve Rot", swerve.getSwerveDrive().getPose().getRotation().toString());
        SmartDashboard.putNumber("TEST", swerve.getGyroRot().getAngle());

    }

    /**
     * Runs when first entering Autonomous mode
     */

    Timer timer = new Timer();
    @Override
    public void autonomousInit() {
        if (this.auto != null) {
            if (this.auto.getSelected() != null)
                this.auto.getSelected().initialize();
        }

        timer.reset();
        timer.start();
    }

    @Override
    public void disabledPeriodic() {
        elevator.moveToState(Elevator.ElevatorState.ZERO);
        arm.moveToState(Arm.ArmState.ZERO);
    }

    /**
     * Runs every 20ms during Autonomous
     */
    @Override
    public void autonomousPeriodic() {
        if(this.auto.getSelected() != null) {
            if(this.auto.getSelected().isFinished()) {
                return;
            }
            this.auto.getSelected().execute();
        }

/*
        if(!timer.hasElapsed(2)) {
            swerve.move(-0.9,0, 0);
        }
        else {
            swerve.move(0,0,0);
        }
 */
    }

    /**
     * Runs at the start of Teleop state
     */
    @Override
    public void teleopInit() {
    }

    /**
     * Runs every 20ms of Teleop
     */
    @Override
    public void teleopPeriodic() {}

    /**
     * Runs at the start of Test state
     */
    double setpoint = Units.inchesToMeters(36);
    @Override
    public void testInit() {


/*        swerve.getSwerveDrive().resetOdometry(new Pose2d());

                relocate = false;
                reangle = false;
                reposition = false;
                strafe = false;
    }
    private void move(double xSpeed, double ySpeed, double rot) {
        swerve.drive(new Translation2d(xSpeed, ySpeed), rot,
                false,
                true, Constants.BOT_CENTER);
  */

/*
        kinematics = swerve.getSwerveDrive().kinematics;

        PIDController xController = new PIDController(4.157,0,0);
        PIDController yController = new PIDController(4.157,0,0);
        ProfiledPIDController thetaController = new ProfiledPIDController(1,0,0, new TrapezoidProfile.Constraints(Math.PI, Math.PI)
        );
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
        dController = new HolonomicDriveController(xController, yController, thetaController);
 */
    }
    boolean relocate = false; //move forward 1 ft
     boolean reangle = false; //rotate towards april tag
     boolean reposition = false; //move towards april tag
     boolean strafe = false; // Move left or right to center april tag



    /**
     * Runs every 20ms of Test
     */
    @Override
    public void testPeriodic() {

//        swerve.setTargetPose(new Pose2d(0.6096, 0.6096, Rotation2d.fromDegrees(90)));
//swerve.getSwerveDrive().setChassisSpeeds(new ChassisSpeeds(0.4,0.2,Units.degreesToRadians(15)));

//        swerve.getSwerveDrive().setChassisSpeeds();
/*        if(swerve.getSwerveDrive().getPose().getX() < Units.inchesToMeters(12)) {
            swerve.move(0.1, 0, 0);
        } else {
            swerve.move(0, 0, 0);
        }
*/
//        double yaw = swerve.getSwerveDrive().getYaw().getDegrees();
//        double Xvalue = swerve.getSwerveDrive().getPose().getX();
//        double Yvalue = swerve.getSwerveDrive().getPose().getY();
/*
        if(yaw < 45) {
            swerve.rotate(1);
        } else {
            swerve.rotate(0);
        }
*/

/*
        if(swerve.getSwerveDrive().getPose().getX() < 12) {
            swerve.move(0.1, 0, 0);
        } else {
            swerve.move(0, 0, 0);
        }
*/
       /* if(Yvalue < 12) {
            swerve.move(0, 0.1, 0);
        } else {
            swerve.move(0, 0, 0);
        }
*/




        /*
        for (swervelib.SwerveModule module : this.swerve.getSwerveDrive().getModules()) {
            SmartDashboard.putNumber("Module" + module.moduleNumber + " Encoder", module.getAbsolutePosition());
        }

        var table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("Left");
        boolean hasTarget = table.getEntry("hasTarget").getBoolean(false);double yaw = table.getEntry("targetYaw").getDouble(0.0);//
        //double yaw = vision.getTargetYaw();
        double distY = table.getEntry("targetPixelsY").getDouble(0.0);

        SmartDashboard.putBoolean("Has Target", hasTarget);SmartDashboard.putNumber("Target Yaw", yaw);
        SmartDashboard.putNumber("Target Yaw", yaw);
        //Motor movement

        if(auxController.getBButton() && !reposition) {
            reposition = true;
        }
        double distX = backVision.getDistanceToTarget(0,11,10.3125,false);
        if(reposition && distX != -1) {
            if(!MathUtil.isNear(12, distX, 0.001)) {
                swerve.move(MathUtil.clamp(12-distX*2, -0.1, 0.1), 0,0);
            } else {
                reposition = false;
                swerve.move(0, 0,0);
            }
        }
*/

/*
        if(swerve.getSwerveDrive().getPose().getX() < setpoint) {
            swerve.move(0.1, 0, 0);
        } else {
            swerve.move(0, 0, 0);
        }
*/


      /*  if(driveController.getAButtonPressed()) {
            swerve.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward).schedule();
        } else if(driveController.getBButtonPressed()) {
            swerve.sysIdQuasistaticAngle(SysIdRoutine.Direction.kForward).schedule();
        } else if(driveController.getXButtonPressed()) {
            swerve.sysIdDynamicDrive(SysIdRoutine.Direction.kForward).schedule();
        } else if(driveController.getYButtonPressed()) {
            swerve.sysIdDynamicAngle(SysIdRoutine.Direction.kForward).schedule();
        }

        if(driveController.getPOV() == 0) {
            swerve.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse).schedule();
        } else if(driveController.getPOV() == 90) {
            swerve.sysIdQuasistaticAngle(SysIdRoutine.Direction.kReverse).schedule();
        } else if(driveController.getPOV() == 180) {
            swerve.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse).schedule();
        } else if(driveController.getPOV() == 270) {
            swerve.sysIdDynamicAngle(SysIdRoutine.Direction.kReverse).schedule();
        }

       */
    }
}
