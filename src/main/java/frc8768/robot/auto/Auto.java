package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * Auton example for swerve using PathPlanner
 */
public class Auto {
    private final SendableChooser<Command> autonChooser;
    private final SwerveSubsystem swerve;

    /**
     * Auto constructor, builds everything.
     *
     * @param swerve The Robots swerve subsystem
     */
    public Auto(SwerveSubsystem swerve) {
        SwerveDrive swerveDrive = swerve.getSwerveDrive();
        this.swerve = swerve;

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(0.00, 0.00, 0.01),
                new PIDConstants(0.00, 0.00, 0.01),
                14.5,
                327.025,
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
                () -> true,
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
        return new PathPlannerAuto("Right Platform").andThen(new BalanceChassisCommand(swerve));
    }

    public Command midPlatform() {
        return new PathPlannerAuto("Middle Platform")
                .andThen(new BalanceChassisCommand(swerve));
    }

    public Command leftPlatform() {
        return new PathPlannerAuto("Left Platform")
                .andThen(new BalanceChassisCommand(swerve));
    }

    public Command community() {
        return new PathPlannerAuto("Community");
    }
}
