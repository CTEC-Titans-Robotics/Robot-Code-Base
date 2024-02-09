package frc8768.robot.util;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc8768.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class FollowTrajectory extends Command {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private final Supplier<Rotation2d> m_desiredRotation;
    private final SwerveSubsystem m_driveSubsystem;

    /**
     * Constructs a new FollowTrajectory command that when executed will follow the
     * provided trajectory. This command will not return output voltages but rather
     * raw module states from the position controllers which need to be put into a
     * velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path- this is left to the user, since it is not appropriate for paths 
     * with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param controller      The HolonomicDriveController for the drivetrain.
     * @param desiredRotation The angle that the drivetrain should be facing.
     *                        This is sampled at each
     *                        time step.
     * @param driveSubsystem  The subsystem to use to drive the robot.
     * @param requirements    The subsystems to require.
     */
    public FollowTrajectory(
            Trajectory trajectory,
            HolonomicDriveController controller,
            Supplier<Rotation2d> desiredRotation,
            SwerveSubsystem driveSubsystem,
            Subsystem... requirements) {
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");
        m_driveSubsystem = requireNonNullParam(driveSubsystem, "driveSubsystem", "SwerveControllerCommand");
        m_desiredRotation = requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = m_trajectory.sample(curTime);
        Rotation2d desiredRotation = m_desiredRotation.get();
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_driveSubsystem.getSwerveDrive().getPose(), desiredState, desiredRotation);
        // This is done because the rotation is inverted. 
        // It may not be the same on your robot, so if the rotation does not function as expected, remove this.
        targetChassisSpeeds.omegaRadiansPerSecond *= -1;
        var targetModuleStates = m_driveSubsystem.getSwerveDrive().kinematics.toSwerveModuleStates(targetChassisSpeeds);
        m_driveSubsystem.getSwerveDrive().setModuleStates(targetModuleStates, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}