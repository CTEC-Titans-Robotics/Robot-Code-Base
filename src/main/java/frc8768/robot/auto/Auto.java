package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.auto.commands.L1Command;
import frc8768.robot.auto.commands.L1FullyAuto;
import frc8768.robot.auto.commands.TestTaxi;
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
        NamedCommands.registerCommand("L1_Shoot", new L1Command(arm));

        RobotConfig config = new RobotConfig(
                Kilograms.of(Constants.WEIGHT),
                KilogramSquareMeters.of(Constants.INERTIA),
                new ModuleConfig(
                        Units.inchesToMeters(2),
                        Constants.SwerveConfig.MAX_SPEED,
                        1.19,
                        swerveDrive.swerveDriveConfiguration.getDriveMotorSim(),
                        40,
                        1

                ),
                new Translation2d(Units.inchesToMeters(10.5), Units.inchesToMeters(10.5)),
                new Translation2d(Units.inchesToMeters(10.5),Units.inchesToMeters(-10.5)),
                new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(10.5)),
                new Translation2d(Units.inchesToMeters(-10.5), Units.inchesToMeters(-10.5))
        );

        PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(0.01, 0, 0),
                new PIDConstants(0.01, 0, 0)
        );

        AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                swerveDrive::setChassisSpeeds,
                driveController,
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                }
        );

        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.addOption("Test Taxi", new TestTaxi(swerve));
        autonChooser.addOption("L1 Full Auto", new L1FullyAuto(swerve, arm));
        SmartDashboard.putData("Auto", this.autonChooser);
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
