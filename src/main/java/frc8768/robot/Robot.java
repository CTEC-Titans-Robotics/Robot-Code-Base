// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
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
    private static final XboxController controller = new XboxController(Constants.driverControllerId);

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

        // try {
        //   this.swerve = new SwerveSubsystem(Constants.SwerveConfig.currentType);
        // } catch (IOException io) {
        //   throw new RuntimeException("Swerve failed to create!", io);
        // }

        // this.drivebase = new DrivebaseOperator();
        // this.auto = new Auto(swerve);
        this.vision = new LimelightVision("limelight");

        // drivebase.init();
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
        LogUtil.run();

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

    CANdle LED = new CANdle(3);
    CANSparkFlex flex = new CANSparkFlex(4, MotorType.kBrushless);

    /**
     * Runs at the start of Test state
     */
    @Override
    public void testInit() {
        LED.configV5Enabled(true);
        LED.configLEDType(CANdle.LEDStripType.BRGW);
    }

    /**
     * Runs every "tick" of Test time
     */
    @Override
    public void testPeriodic() {
        LED.setLEDs(255, 0, 0);

        if(controller.getAButtonPressed()) {
            flex.set(0.25);
        } else if(controller.getBButtonPressed()) {
            flex.set(0.5);
        } else if(controller.getYButtonPressed()) {
            flex.set(0.75);
        } else if(controller.getXButtonPressed()) {
            flex.set(1);
        } else {
            flex.set(0);
        }
    }
}
