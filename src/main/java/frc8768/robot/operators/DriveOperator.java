package frc8768.robot.operators;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc8768.robot.subsystems.SwerveSubsystem;
import frc8768.robot.util.Constants;
import swervelib.SwerveInputStream;

public class DriveOperator implements Operator {
    private final CommandXboxController controller = getController();
    private final SwerveSubsystem m_swerve;

    SwerveInputStream driveAngularVelocity;

    public DriveOperator(SwerveSubsystem swerve) {
        m_swerve = swerve;

        driveAngularVelocity = SwerveInputStream.of(m_swerve.getSwerveDrive(),
                        () -> controller.getLeftY() * -1,
                        () -> controller.getLeftX() * -1)
                .withControllerRotationAxis(controller::getRightX)
                .deadband(Constants.CONTROLLER_DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);
    }

    @Override
    public String getName() {
        return "Drive";
    }

    @Override
    public int getControllerId() {
        return Constants.DRIVER_CONTROLLER_ID;
    }

    @Override
    public void configureBindings() {
        Command driveFieldRelative = m_swerve.drive(driveAngularVelocity);
        m_swerve.setDefaultCommand(driveFieldRelative);
    }
}
