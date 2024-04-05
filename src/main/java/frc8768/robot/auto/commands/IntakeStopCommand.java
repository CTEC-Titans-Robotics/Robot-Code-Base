package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.IntakeSubsystem;

public class IntakeStopCommand extends Command {
    private final IntakeSubsystem intake;

    public IntakeStopCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.releaseLock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
