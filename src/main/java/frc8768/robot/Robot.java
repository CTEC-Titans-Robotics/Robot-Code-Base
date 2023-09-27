// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DrivebaseOperator;
import frc8768.robot.operators.PeripheralOperator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.subsystems.TankSubsystemFalcon;
import frc8768.robot.subsystems.TankSubsystemSpark;
import frc8768.robot.util.Constants;
import frc8768.robot.util.MathUtil;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.imu.Pigeon2Swerve;

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
    private DrivebaseOperator drivebase;
    private PeripheralOperator peripheral;
    private SwerveSubsystem swerve;
    private TankSubsystemFalcon falcon;
    private TankSubsystemSpark spark;
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
        CameraServer.startAutomaticCapture();

        try {
            swerve = new SwerveSubsystem(Constants.SwerveConfig.currentType);
        } catch (IOException io) {
            throw new RuntimeException("Swerve failed to create!", io);
        }

        drivebase = new DrivebaseOperator();
        peripheral = new PeripheralOperator();

        this.auto = new Auto(swerve);
        drivebase.init();
        peripheral.init();
    }

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
    
    @Override
    public void robotPeriodic() {
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
    public void testPeriodic() {
        for(SwerveModule module : swerve.getSwerveDrive().swerveDriveConfiguration.modules) {
            System.out.printf("Module %d has pos: %s\n", module.moduleNumber, module.getAbsolutePosition());
        }
    }
    
    
    @Override
    public void simulationInit() {}
    
    
    @Override
    public void simulationPeriodic() {}
}
