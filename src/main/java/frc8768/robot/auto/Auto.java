package frc8768.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * Auton example for swerve using PathPlanner
 */
public class Auto {
    private final SendableChooser<Command> autonChooser;

    /**
     * Auto constructor, builds everything.
     *
     * @param swerve The Robots swerve subsystem
     */
    public Auto(SwerveSubsystem swerve) {
        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(0.00, 0.00, 0.01),
                new PIDConstants(0.00, 0.00, 0.01),
                // TODO: Put max module speed here
                14.2,
                // TODO: Put your robot chassis radius here
                5,
                new ReplanningConfig(
                        false,
                        true
                )

        );

        AutoBuilder.configureHolonomic(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                swerveDrive::setChassisSpeeds,
                config,
                swerve);

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("No-op", new InstantCommand());
        autonChooser.addOption("Left Platform", leftPlatform());
        autonChooser.addOption("Middle Platform", midPlatform());
        autonChooser.addOption("Right Platform", rightPlatform());
        autonChooser.addOption("Community", community());

        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    /**
     * Get the current Auton mode
     *
     * @return Current Auton Mode
     */
    public Command getSelected() {
        return autonChooser.getSelected();
    }

    public Command rightPlatform() {
        return builder.fullAuto(PathPlanner.loadPath("Right Platform", new PathConstraints(7.25, 5)))
                .andThen(new BalanceChassisCommand(swerve));
    }

    public Command midPlatform() {
        return builder.fullAuto(PathPlanner.loadPath("Middle Platform", new PathConstraints(7.25, 5)))
                .andThen(new BalanceChassisCommand(swerve));
    }

    public Command leftPlatform() {
        return builder.fullAuto(PathPlanner.loadPath("Left Platform", new PathConstraints(7.25, 5)))
                .andThen(new BalanceChassisCommand(swerve));
    }

    public Command community() {
        return builder.fullAuto(PathPlanner.loadPath("Community", new PathConstraints(7.25, 5)));
    }
}
