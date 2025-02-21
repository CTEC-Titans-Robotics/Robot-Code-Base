// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.Elevator;
import frc8768.robot.subsystems.GroundIndefector;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.Vision;

import java.io.IOException;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private static final XboxController driveController = new XboxController(Constants.DRIVER_CONTROLLER_ID);

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
    private GroundIndefector groundIndefector;
    private Elevator elevator;
    // private TankSubsystemFalcon falcon;
    // private TankSubsystemSpark spark;

    /**
     * Vision API instance
     */
    public LimelightVision vision;

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

        try {
          this.swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }

        this.groundIndefector = new GroundIndefector();
        this.elevator = new Elevator();

        this.drivebase = new DrivebaseOperator(driveController, this.swerve, this.groundIndefector);
        // this.auto = new Auto(swerve);
        // this.vision = new LimelightVision("limelight");

        this.drivebase.init();
    }

    /* For tank
    public TankSubsystemFalcon getFalcon() {
        return this.falcon;
    }
    public TankSubsystemSpark getSpark() {
        return this.spark;
    }
     */

    public Vision getVision() {
        return this.vision;
    }

    /**
     * Runs even if the Robot is disabled.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        LogUtil.run();
    }

    /**
     * Runs when first entering Autonomous mode
     */
    @Override
    public void autonomousInit() {
        if (this.auto != null) {
            this.auto.getSelected().schedule();
        }
    }

    /**
     * Runs every 20ms during Autonomous
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * Runs at the start of Teleop state
     */
    @Override
    public void teleopInit() {}

    /**
     * Runs every 20ms of Teleop
     */
    @Override
    public void teleopPeriodic() {}

    /**
     * Runs at the start of Test state
     */
    @Override
    public void testInit() {}

    /**
     * Runs every 20ms of Test
     */
    @Override
    public void testPeriodic() {
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
        if(driveController.getYButton()) {
            elevator.up();
        } else if (driveController.getAButton()) {
            elevator.down();

        } else {
            elevator.stop();
        }

    }
}
