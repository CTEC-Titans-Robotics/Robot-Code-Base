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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import swervelib.SwerveDrive;

/**
 * Auton example for swerve using PathPlanner
 */
public class Auto {

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    private final SwerveSubsystem swerve;

    public SendableChooser<Command> getAutonChooser() {
        return autonChooser;
    }

    /**
     * Auto constructor, builds everything.
     *
     * @param swerve The Robots swerve subsystem
     *
     */
    public Auto(SwerveSubsystem swerve) {
        this.swerve = swerve;
        SwerveDrive swerveDrive = this.swerve.getSwerveDrive();

        //Configure Drivebase for following paths

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(0.045849, 0.00, 0.0068069),
                new PIDConstants(0.0091857, 0.00, 0.00052987),
                Constants.SwerveConfig.MAX_SPEED,
                0.267,
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
                () -> DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                swerve);

        //select an auton into the auton chooser so it can be executed in the robot class

        autonChooser.setDefaultOption("No-Op", new InstantCommand());
        autonChooser.addOption("Test Auto", AutoBuilder.buildAuto("Test Auto"));

        SmartDashboard.putData("Auto", autonChooser);

        //END OF CONTRUCTOR
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
        return null;
    }
//Accessor function: Gets the currently selected auton so that it can be executed in the robot class
};
