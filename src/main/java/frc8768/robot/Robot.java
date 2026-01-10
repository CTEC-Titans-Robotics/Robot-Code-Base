// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc8768.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    public final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        super.robotInit();

        m_robotContainer.onRobotInit();
    }

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
        Command m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
