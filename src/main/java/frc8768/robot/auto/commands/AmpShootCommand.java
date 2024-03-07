package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;

public class AmpShootCommand extends Command {
    private IntakeSubsystem intake;

    public AmpShootCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        this.intake.beginStage(IntakeSubsystem.IntakeStage.AMP);
    }
}
