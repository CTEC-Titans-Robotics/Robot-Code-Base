package frc8768.robot.auto.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc8768.robot.subsystems.Arm;
public class L4Arm extends Command{

    private final Arm arm;


    public L4Arm(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.arm.moveToState(Arm.ArmState.L4);

    }
}
