package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class Shoot extends Command{

    private final Arm arm;
    private final Timer timer = new Timer();
    private final Elevator elevator;

    public Shoot(Elevator elevator, Arm arm) {
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


        timer.start();
        arm.spinIntake(true);




            /*elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.stopIntake();

             */
        }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
    }


