// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import swervelib.SwerveDrive;

import java.io.IOException;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    public static Robot instance;
    private final DrivebaseOperator drivebase = new DrivebaseOperator();
    private SwerveSubsystem swerve;
    private Auto auto;

    public static Robot getInstance() {
        return instance;
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        instance = this;

        try {
            swerve = new SwerveSubsystem(Constants.SwerveConfig.currentType);
        } catch (IOException io) {
            throw new RuntimeException("Swerve failed to create!", io);
        }

        this.auto = new Auto(swerve);
        drivebase.init();
    }

    public SwerveSubsystem getSwerve() {
        return this.swerve;
    }
    
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void autonomousInit() {
        if (auto != null) {
            auto.getSelected().schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void teleopInit() {}
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}
    
    
    @Override
    public void testInit() {}
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void simulationInit() {}
    
    
    @Override
    public void simulationPeriodic() {}
}
