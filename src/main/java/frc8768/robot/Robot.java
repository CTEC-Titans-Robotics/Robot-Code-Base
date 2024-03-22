// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.AuxiliaryOperator;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.robot.util.MathUtil;
import frc8768.visionlib.PhotonVision;
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
    public static boolean goUp = false;

    /**
     * Drivebase Operator
     */
    private DrivebaseOperator drivebase;

    /**
     * Auxiliary Operator, handles all non-drivebase telelop functions.
     */
    private AuxiliaryOperator auxiliary;

    private PhotonVision leftVision;
    private PhotonVision rightVision;

    /**
     * The swerve subsystem, held in here for Auton.
     */
    private SwerveSubsystem swerve;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    // private TankSubsystemFalcon falcon;
    // private TankSubsystemSpark spark;

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

    public PhotonVision getLeftVision() {
        return leftVision;
    }

    public PhotonVision getRightVision() {
        return rightVision;
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture(0);

        this.leftVision = new PhotonVision("Left");
        // this.rightVision = new PhotonVision("Right");

        // Subsystem init
        try {
          this.swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException io) {
          throw new RuntimeException("Swerve failed to create!", io);
        }
        this.arm = new ArmSubsystem();
        this.intake = new IntakeSubsystem();

        this.auto = new Auto(this.swerve, this.arm, this.intake);

        // Operator creation
        this.drivebase = new DrivebaseOperator(this.swerve, this.arm, this.intake);
        this.auxiliary = new AuxiliaryOperator(this.arm, this.intake, this.leftVision);

        // this.vision = new LimelightVision("limelight");

        // Init
        this.drivebase.init();
        this.auxiliary.init();

        // Init logging
        LogUtil.registerDashLogger(this.arm::dashboard);
        LogUtil.registerDashLogger(this.intake::dashboard);
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

        if(this.drivebase != null && !this.drivebase.isAlive()) {
            LogUtil.LOGGER.log(Level.WARNING, "Drivebase thread died! Reviving...");
            this.drivebase.reviveThread();
        }
        if(this.auxiliary != null && !this.auxiliary.isAlive()) {
            LogUtil.LOGGER.log(Level.WARNING, "Auxiliary thread died! Reviving...");
            this.auxiliary.reviveThread();
        }

        this.arm.tick();
        this.intake.tick();

        LogUtil.run();
    }

    /**
     * Runs when first entering Autonomous mode
     */
    @Override
    public void autonomousInit() {
        if (this.auto != null) {
            if(!this.auto.getSelected().getName().contains("Block")) {
                this.swerve.autonInit();
            }
            this.auto.getSelected().schedule();
        }
    }

    /**
     * Runs every "tick" of Autonomous time
     */
    @Override
    public void autonomousPeriodic() {
        if(this.auto != null) {
            if(!this.auto.getSelected().isScheduled()) {
                return;
            }
            //this.auto.getSelected().execute();
        }
    }

    /**
     * Runs at the start of Teleop state
     */
    @Override
    public void teleopInit() {
        this.drivebase.initTeleop();
    }

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
    public void testPeriodic() {
        for(swervelib.SwerveModule module : this.swerve.getSwerveDrive().getModules()) {
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Encoder", module.getAbsolutePosition());
        }

        /* PID Tuning
                this.swerve.getSwerveDrive().drive(Constants.BOT_CENTER, MathUtil.getRadFromDeg(90), true, false, Constants.BOT_CENTER);
         */
    }
}
