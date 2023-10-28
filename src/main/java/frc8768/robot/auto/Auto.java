package frc8768.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.util.HashMap;

/**
 * Auton example for swerve using PathPlanner
 */
public class Auto {
    public static final PathConstraints CONSTRAINTS = new PathConstraints(7.25, 5);

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
        autonChooser.addOption("Right Platform", rightPlatform());
        autonChooser.addOption("Left Platform", leftPlatform());
        autonChooser.addOption("Middle Platform", middlePlatform());
        autonChooser.addOption("Community", community());
        
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    public Command rightPlatform() {
        return builder.fullAuto(
                PathPlanner.loadPath("Right Platform", CONSTRAINTS)).withName("platform");
      }

      public Command leftPlatform() {
        return builder.fullAuto(
                PathPlanner.loadPath("Left Platform", CONSTRAINTS)).withName("platform");
      }

      public Command middlePlatform() {
        return builder.fullAuto(
                PathPlanner.loadPath("Middle Platform", CONSTRAINTS)).withName("platform");
      }

      public Command community() {
        return builder.fullAuto(
                PathPlanner.loadPath("Community", CONSTRAINTS));
      }

      public Command levelOut() {
        return new BalanceChassisCommand(swerve);
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
