package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class Shoot extends Command{

    private final Arm arm;
    private final Elevator elevator;

    public Shoot(Elevator elevator, Arm arm) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
            arm.spinIntake(true);
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.stopIntake();
        }
    }


