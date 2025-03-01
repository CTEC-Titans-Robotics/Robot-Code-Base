package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.Arm;
public class L1Command extends Command{

    private final Arm arm;
    private final Timer timer = new Timer();

    public L1Command(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.arm.moveToState(Arm.ArmState.L1);

        if(arm.isAtRotation()){
            if (!timer.isRunning()) {
                timer.reset();
                timer.start();
            }
            arm.spinIntake(true);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }
}
