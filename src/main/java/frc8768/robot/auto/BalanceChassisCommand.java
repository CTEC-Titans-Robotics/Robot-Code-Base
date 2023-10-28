package frc8768.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc8768.robot.subsystems.SwerveSubsystem;


public class BalanceChassisCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private PIDController pid;

	public BalanceChassisCommand(SwerveSubsystem swerve) {
		this.pid = new PIDController(BalanceChassisConstants.kP, BalanceChassisConstants.kI, BalanceChassisConstants.kD);
		this.pid.setSetpoint(BalanceChassisConstants.kGroundAngle);
		this.pid.setTolerance(BalanceChassisConstants.kTolerance);

		this.swerve = swerve;
		this.addRequirements(this.swerve);
	}

	@Override
	public void execute() {
		double vxMeters = MathUtil.clamp(this.pid.calculate(frc8768.robot.util.MathUtil.getRadFromDeg(this.swerve.getGyroRot().getAngle())),
				-BalanceChassisConstants.kDriveSpeedMPS, BalanceChassisConstants.kDriveSpeedMPS);

		this.swerve.drive(new Translation2d(-vxMeters, 0), 0, false, false, false);
	}

	@Override
	public boolean isFinished() {
		return this.pid.atSetpoint();
	}
}