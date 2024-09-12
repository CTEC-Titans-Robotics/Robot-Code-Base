package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    public Auto(SwerveSubsystem swerve) {
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
                )
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
                () -> false, // Change if needed
                swerve);

        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("No-op", new InstantCommand());
    }



    /**
     * Get the current Auton mode
     *
     * @return Current Auton Mode
     */
    public void Translation(double x , double y ) {
        /*
        if(auto != null) {
            if(!auto.getSelected().isScheduled()) {
                return;
            }
            auto.getSelected().execute();
        }
        */

            // Get the current pose from the odometry
        Pose2d currentPose = swerve.getSwerveDrive().getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        // Drive forward if the current distance is less than the target distance
        if (currentX < x) {
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(1.0, 0.0, 0.0)); // 1.0 m/s forward, no strafing, no rotation
        }

        if (currentY < y) {
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(0.0, 1.0, 0.0)); // 1.0 m/s forward, no strafing, no rotation
        } else {
            // Stop the robot once the target distance is reached
            swerve.getSwerveDrive().drive(
                    new ChassisSpeeds(0.0, 0.0, 0.0));
        }
    }


    public Command getSelected() {
        return autonChooser.getSelected();
    }
}
