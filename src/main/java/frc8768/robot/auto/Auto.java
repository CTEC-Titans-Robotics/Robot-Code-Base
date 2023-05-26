package frc8768.robot.auto;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.util.HashMap;

public class Auto {
    private SwerveSubsystem swerve;
    private SwerveAutoBuilder builder;
    private final SendableChooser<Command> autonChooser;
    private final HashMap<String, Command> eventMap;

    public Auto(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.eventMap = new HashMap<>();

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
        autonChooser.addOption("No-op", new InstantCommand());
    }

    public Command getSelected() {
        return autonChooser.getSelected();
    }
}
