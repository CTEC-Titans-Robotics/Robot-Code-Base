package frc8768.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.util.HashMap;

/**
 * Auton example for swerve using PathPlanner
 */
public class Auto {
    private SwerveSubsystem swerve;
    private SwerveAutoBuilder builder;
    private final SendableChooser<Command> autonChooser;
    private final HashMap<String, Command> eventMap;

    /**
     * Auto constructor, builds everything.
     *
     * @param swerve The Robots swerve subsystem
     */
    public Auto(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.eventMap = new HashMap<>();
        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        this.builder = new SwerveAutoBuilder(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                new PIDConstants(0.045849, 0.0, 0.0068069),
                new PIDConstants(0.0091857, 0.0, 0.00052987),
                swerveDrive::setChassisSpeeds,
                eventMap,
                true,
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
