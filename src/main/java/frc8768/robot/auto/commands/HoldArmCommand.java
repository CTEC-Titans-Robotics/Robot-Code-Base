package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;

public class HoldArmCommand extends Command {
    private final ArmSubsystem arm;
    private final Timer timer = new Timer();

    public HoldArmCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        this.arm.setDesiredState(ArmSubsystem.ArmState.LOW);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.05);
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.releaseLock();
        super.end(interrupted);
    }
}
