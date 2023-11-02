// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.operators.IntakeOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.subsystems.TankSubsystemFalcon;
import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;
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
    /**
     * Robot instance, can't be seen across threads
     */
    public static Robot instance;

    /**
     * Drivebase Operator
     */
    private DrivebaseOperator drivebase;
    private IntakeOperator intake;

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

        drivebase = new DrivebaseOperator();

        try {
          swerve = new SwerveSubsystem(Constants.SwerveConfig.currentType);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }


        this.auto = new Auto(swerve);
        this.vision = new Vision(Vision.Type.PI);

        drivebase.init();
        intake.init();
    }

    /**
     * Get the swerve subsystem, often for auton mechanics.
     *
     * @return The swerve subsystem
     */
    public SwerveSubsystem getSwerve() {
        return this.swerve;
    }
    /* For tank
    public TankSubsystemFalcon getFalcon() {
        return this.falcon;
    }
    public TankSubsystemSpark getSpark() {
        return this.spark;
    }
     */

    /**
     * Runs even if the Robot is disabled.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Runs when first entering Autonomous mode
     */
    @Override
    public void autonomousInit() {
        if (auto != null) {
            auto.getSelected().schedule();
        }
    }

    /**
     * Runs every "tick" of Autonomous time
     */
    @Override
    public void autonomousPeriodic() {
        if(auto != null) {
            if(!auto.getSelected().isScheduled()) {
                return;
            }
            auto.getSelected().execute();
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
    public void teleopPeriodic() {}

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
