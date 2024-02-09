package frc8768.robot.util;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc8768.robot.subsystems.SwerveSubsystem;
import swervelib.parser.PIDFConfig;

public class SwerveUtil {
    public static HolonomicDriveController constructHolonomicFromSwerve(SwerveSubsystem swerve) {
        PIDFConfig headingConf = swerve.getSwerveDrive().getSwerveController().config.headingPIDF;
        PIDFConfig translationError = new PIDFConfig(0.045849, 0, 0.0068069);
        return new HolonomicDriveController(translationError.createPIDController(), translationError.createPIDController(),
                new ProfiledPIDController(headingConf.p, headingConf.i, headingConf.d,
                        new TrapezoidProfile.Constraints(swerve.getSwerveDrive().getSwerveController().config.maxAngularVelocity, MathUtil.getRadFromDeg(450))));
    }
}
