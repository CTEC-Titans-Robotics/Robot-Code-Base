package frc8768.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc8768.robot.subsystems.SwerveSubsystem;

public class SysIdUtil {
    private static SysIdRoutine routine;

    public static void createAngleRoutine(SwerveSubsystem swerve) {
        SysIdRoutine.Config cfg = new SysIdRoutine.Config();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(swerve::voltAngleForward, swerve::logAngle, swerve);
        routine = new SysIdRoutine(cfg, mechanism);
    }

    public static void createDriveRoutine(SwerveSubsystem swerve) {
        SysIdRoutine.Config cfg = new SysIdRoutine.Config();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(swerve::voltDriveForward, swerve::logDrive, swerve);
        routine = new SysIdRoutine(cfg, mechanism);
    }

    public static Command runSysIdQuasistatic() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public static Command runSysIdDynamic() {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }
}