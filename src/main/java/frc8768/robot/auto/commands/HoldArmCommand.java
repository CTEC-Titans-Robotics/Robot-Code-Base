package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;

public class HoldArmCommand extends Command {
    private final ArmSubsystem arm;

    public HoldArmCommand(ArmSubsystem arm) {
        this.arm = arm;
        Robot.goUp = true;
    }

    @Override
    public void execute() {
        if(!isFinished()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.LOW);
        } else {
            this.arm.releaseLock();
        }
    }

    @Override
    public boolean isFinished() {
        return !Robot.goUp || Robot.getInstance().isTeleopEnabled();
    }
}
