package frc8768.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        PIDController theta = swerve.getSwerveDrive().swerveController.thetaController;

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
                new PIDConstants(0.01, 0.0, 0.0),
                new PIDConstants(theta.getP(), theta.getI(), theta.getD()),
                5.18,
                0.26035,
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

        //setup commands so that they can be used in path planner

        NamedCommands.registerCommand("hold_arm_drive", new HoldArmCommand(arm));
        NamedCommands.registerCommand("amp_shoot", new AmpShootCommand(intake));
        NamedCommands.registerCommand("amp_raise", new AmpRaiseCommand(arm));
        NamedCommands.registerCommand("drop_arm", new DropArmCommand(arm, intake));
        NamedCommands.registerCommand("stop_intake", new IntakeStopCommand(intake));
        NamedCommands.registerCommand("speaker_shoot", new SpeakerShootCommand(arm, intake));
        //TODO: What is trying to call this?
        NamedCommands.registerCommand("speaker_far_shoot", new SpeakerShootCommand(arm, intake));

        // Setup Positions for Shuffleboard selector

        this.autonChooser = new SendableChooser<>();
        this.autonChooser.setDefaultOption("No-Op", new InstantCommand().andThen(arm::releaseLock));

        this.autonChooser.addOption("Position 1 Disruptor", AutoBuilder.buildAuto("Position_1_Block"));
        this.autonChooser.addOption("Position 5 Disruptor", AutoBuilder.buildAuto("Position_5_Block"));

        // this.autonChooser.addOption("Omega", AutoBuilder.buildAuto("Omega"));

        this.autonChooser.addOption("Position 1", AutoBuilder.buildAuto("Position_1"));
        this.autonChooser.addOption("Position 5", AutoBuilder.buildAuto("Position_5"));

        this.autonChooser.addOption("Position 2 Shoot and Move", AutoBuilder.buildAuto("Position_2"));
        this.autonChooser.addOption("Position 3 Shoot and Move", AutoBuilder.buildAuto("Position_3"));
        this.autonChooser.addOption("Position 4 Shoot and Move", AutoBuilder.buildAuto("Position_4"));

        this.autonChooser.addOption("Position 2 Shoot Only", AutoBuilder.buildAuto("Position_2_Shoot"));
        this.autonChooser.addOption("Position 3 Shoot Only", AutoBuilder.buildAuto("Position_3_Shoot"));
        this.autonChooser.addOption("Position 4 Shoot Only", AutoBuilder.buildAuto("Position_4_Shoot"));

        this.autonChooser.addOption("Position 2 MultiNote", AutoBuilder.buildAuto("Position_2_MNote"));
        this.autonChooser.addOption("Position 3 MultiNote", AutoBuilder.buildAuto("Position_3_MNote"));
        this.autonChooser.addOption("Position 4 MultiNote", AutoBuilder.buildAuto("Position_4_MNote"));
        this.autonChooser.addOption("Test Open Loop", AutoBuilder.buildAuto("Testing_Open_Loop"));

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
