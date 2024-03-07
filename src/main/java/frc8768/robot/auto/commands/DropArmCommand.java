package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;

public class DropArmCommand extends Command {
    private final ArmSubsystem arm;

    public DropArmCommand(ArmSubsystem arm) {
        this.arm = arm;
        Robot.goUp = false;
    }

    @Override
    public void execute() {
        if(!isFinished()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
        } else {
            this.arm.releaseLock();
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.goUp;
    }
}
