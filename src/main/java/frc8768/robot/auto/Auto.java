package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.auto.commands.*;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
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
    public Auto(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(12, 0.00, 0.032),
                new PIDConstants(34.5, 0.00, 0.59161),
                Constants.SwerveConfig.MAX_SPEED,
                0.268438249,
                new ReplanningConfig(
                        true,
                        true
                )
        );

        AutoBuilder.configureHolonomic(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                swerveDrive::setChassisSpeeds,
                config,
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red, // Change if needed
                swerve);

        this.autonChooser = new SendableChooser<>();
        this.autonChooser.setDefaultOption("No-op", new InstantCommand());

        NamedCommands.registerCommand("hold_arm_drive", new HoldArmCommand(arm));
        NamedCommands.registerCommand("amp_shoot", new AmpShootCommand(intake));
        NamedCommands.registerCommand("amp_raise", new AmpRaiseCommand(arm));
        NamedCommands.registerCommand("drop_arm", new DropArmCommand(arm));
        NamedCommands.registerCommand("speaker_shoot", new SpeakerShootCommand(arm, intake));

        // Setup Positions
        this.autonChooser.addOption("Position 1 Block", AutoBuilder.buildAuto("Position 1 Block"));
        this.autonChooser.addOption("Position 2 Block", AutoBuilder.buildAuto("Position 2 Block"));
        this.autonChooser.addOption("Position 3 Block", AutoBuilder.buildAuto("Position 3 Block"));
        this.autonChooser.addOption("Position 4 Block", AutoBuilder.buildAuto("Position 4 Block"));
        this.autonChooser.addOption("Position 5 Block", AutoBuilder.buildAuto("Position 5 Block"));

        this.autonChooser.addOption("Position 1", AutoBuilder.buildAuto("Position 1"));
        this.autonChooser.addOption("Position 2", AutoBuilder.buildAuto("Position 2"));
        this.autonChooser.addOption("Position 3 N1", AutoBuilder.buildAuto("Position 3 N1"));
        this.autonChooser.addOption("Position 3 N2", AutoBuilder.buildAuto("Position 3 N2"));
        this.autonChooser.addOption("Position 3 N3", AutoBuilder.buildAuto("Position 3 N3"));
        this.autonChooser.addOption("Position 4 N2", AutoBuilder.buildAuto("Position 4 N2"));
        this.autonChooser.addOption("Position 4 N3", AutoBuilder.buildAuto("Position 4 N3"));
        this.autonChooser.addOption("Position 5", AutoBuilder.buildAuto("Position 5"));
        this.autonChooser.addOption("Position 6", AutoBuilder.buildAuto("Position 6"));
    }

    /**
     * Get the current Auton mode
     *
     * @return Current Auton Mode
     */
    public Command getSelected() {
        return autonChooser.getSelected();
    }
}
