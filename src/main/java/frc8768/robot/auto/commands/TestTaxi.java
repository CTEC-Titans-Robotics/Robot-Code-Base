package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.SwerveSubsystem;


public class TestTaxi extends Command{

    Timer timer = new Timer();

    private SwerveSubsystem swerve;

    public TestTaxi(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        swerve.move(-0.15,0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.move(0,0, 0);
        super.end(interrupted);
    }
}
