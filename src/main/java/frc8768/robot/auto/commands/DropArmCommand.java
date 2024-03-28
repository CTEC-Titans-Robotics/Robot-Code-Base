package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;

public class DropArmCommand extends Command {
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();

    public DropArmCommand(ArmSubsystem arm, IntakeSubsystem intake) {
        this.arm = arm;
        this.intake = intake;
        Robot.goUp = false;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
        if(this.arm.getPosition() < 8) {
            this.intake.beginStage(IntakeSubsystem.IntakeStage.INTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.releaseLock();
        this.intake.releaseLock();
        super.end(interrupted);
    }
}
