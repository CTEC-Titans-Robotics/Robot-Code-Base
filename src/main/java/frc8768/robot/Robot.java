// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.Vision;

import java.io.IOException;
import java.util.logging.Level;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
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
    // private TankSubsystemFalcon falcon;
    // private TankSubsystemSpark spark;

    /**
     * Vision API instance
     */
    public Vision vision;

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
          swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }


        this.drivebase = new DrivebaseOperator(this.swerve);
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
        /* if (this.auto != null) {
            this.auto.getSelected().schedule();
        }  */
        swerve.getSwerveDrive().resetOdometry(new Pose2d());

    }

    // Target distance in meters (1 foot = 0.3048 meters)
    double targetDistance = 0.3048;
    @Override
    public void autonomousPeriodic() {
        /*
        if(this.auto != null) {
            if(!this.auto.getSelected().isScheduled()) {
                return;
            }
            this.auto.getSelected().execute();
        }
        */

        // Get the current pose from the odometry
        Pose2d currentPose = swerve.getSwerveDrive().getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        // Drive forward if the current distance is less than the target distance
        if (currentX < targetDistance) {
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(1.0, 0.0, 0.0) // 1.0 m/s forward, no strafing, no rotation
            );
        } else if (currentY < targetDistance) {
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(0.0, 1.0, 0.0) // 1.0 m/s forward, no strafing, no rotation
            );
        } else {
            // Stop the robot once the target distance is reached
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(0.0, 0.0, 0.0)
            );
        }
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
    public void testInit() {}

    /**
     * Runs every "tick" of Test time
     */
    @Override
    public void testPeriodic() {}
}
