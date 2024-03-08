package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;

public class AmpRaiseCommand extends Command {
    private ArmSubsystem arm;

    public AmpRaiseCommand(ArmSubsystem arm) {
        this.arm = arm;
        Robot.goUp = true;
    }

    @Override
    public void execute() {
        this.arm.setDesiredState(ArmSubsystem.ArmState.AMP);
    }
}
