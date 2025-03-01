package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.auto.commands.L1Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;

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
    public Auto(SwerveSubsystem swerve, Arm arm) {
        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        RobotConfig config = new RobotConfig(
                Kilograms.of(Constants.WEIGHT),
                KilogramSquareMeters.of(Constants.INERTIA),
                new ModuleConfig(
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        Constants.SwerveConfig.MAX_SPEED,
                        1.0,
                        swerveDrive.swerveDriveConfiguration.getDriveMotorSim(),
                        30,
                        1

                ),
                new Translation2d(.267, .267),
                new Translation2d(.267,-.273),
                new Translation2d(-.267, .267),
                new Translation2d(-.267, -.267)

        );

        PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(0.01, 0, 0),
                new PIDConstants(0.01, 0, 0),
                0.02
        );

        AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                swerveDrive::setChassisSpeeds,
                driveController,
                config,
                () -> false
        );

        NamedCommands.registerCommand("L1_Shoot", new L1Command(arm));

        autonChooser = new SendableChooser<>();
        autonChooser.addOption("Taxi", AutoBuilder.buildAuto("taxi"));
        autonChooser.addOption("2 (outside) L1", AutoBuilder.buildAuto("2 (outside) L1"));
        autonChooser.addOption("4 (middle) L1", AutoBuilder.buildAuto("4 (middle) L1"));
        autonChooser.addOption("6 (inside) L1", AutoBuilder.buildAuto("6 (inside) L1"));
        autonChooser.setDefaultOption("No-op", new InstantCommand());
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
