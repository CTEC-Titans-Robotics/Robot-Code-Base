package frc8768.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    public Auto(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        this.swerve = swerve;
        this.eventMap = new HashMap<>();
        this.eventMap.put("droparm", new InstantCommand(arm::down));
        this.eventMap.put("armup", new InstantCommand(arm::up));
        this.eventMap.put("dropcube", new InstantCommand(intake::outtake));

        SwerveDrive swerveDrive = swerve.getSwerveDrive();

        this.builder = new SwerveAutoBuilder(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                new PIDConstants(0.01, 0.0, 0.0),
                new PIDConstants(0.01, 0.0, 0.0),
                swerveDrive::setChassisSpeeds,
                eventMap,
                true,
                swerve);

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("No-op", new InstantCommand());
        autonChooser.setDefaultOption("Left Platform", leftPlatform());
        autonChooser.setDefaultOption("Middle Platform", midPlatform());
        autonChooser.setDefaultOption("Right Platform", rightPlatform());
        autonChooser.setDefaultOption("Community", community());
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
        return builder.fullAuto(PathPlanner.loadPath("Mid Platform", new PathConstraints(7.25, 5)))
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
