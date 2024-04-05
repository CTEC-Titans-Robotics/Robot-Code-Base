package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;

public class AmpRaiseCommand extends Command {
    private ArmSubsystem arm;

    public AmpRaiseCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        this.arm.setDesiredState(ArmSubsystem.ArmState.IDLE_AMP);
    }

    @Override
    public boolean isFinished() {
        return this.arm.currState.isAngleWithinFineTolerance(this.arm.getPosition());
    }

    @Override
    public void end(boolean interrupted) {
        this.arm.releaseLock();
        super.end(interrupted);
    }
}
