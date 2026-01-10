package frc8768.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.auto.Auto;
import frc8768.robot.operators.DriveOperator;
import frc8768.robot.operators.Operator;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import frc8768.visionlib.LimelightVision;

import java.io.IOException;

public class RobotContainer {

    /**
     * Driver Operator
     */
    private final Operator m_driveOperator;

    /**
     * Vision API instance
     */
    // public final LimelightVision vision;

    /**
     * Auton Instance
     */
    private final Auto auto;

    public RobotContainer() {
        SwerveSubsystem m_swerve;
        try {
            m_swerve = new SwerveSubsystem(Constants.SwerveConfig.CURRENT_TYPE);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        m_driveOperator = new DriveOperator(m_swerve);

        auto = new Auto(m_swerve);
    }

    public void onRobotInit() {
        m_driveOperator.configureBindings();
    }

    public Command getAutonomousCommand() {
        return auto.getSelected();
    }
}
