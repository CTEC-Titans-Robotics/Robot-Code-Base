package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;

public class SpeakerShootCommand extends Command {
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public SpeakerShootCommand(ArmSubsystem arm, IntakeSubsystem intake) {
        this.arm = arm;
        this.intake = intake;
        Robot.goUp = true;
    }

    @Override
    public void execute() {
        this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        this.intake.beginStage(IntakeSubsystem.IntakeStage.SPEAKER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
        this.intake.releaseLock();
        super.end(interrupted);
    }
}
