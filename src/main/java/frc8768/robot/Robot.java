// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
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
    public void testPeriodic() {}
}
