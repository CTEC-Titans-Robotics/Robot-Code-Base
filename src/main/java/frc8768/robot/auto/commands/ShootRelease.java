package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class ShootRelease extends Command{

    private final Arm arm;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    public ShootRelease(Elevator elevator, Arm arm) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {


        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
           // arm.spinIntake(true);
        timer.start();
            arm.stopIntake();
            elevator.moveToState(Elevator.ElevatorState.ZERO);



        }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
    }


