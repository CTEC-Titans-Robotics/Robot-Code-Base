package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Elevator;
public class L3Elevator extends Command {
    private final Elevator elevator;

    public L3Elevator(Elevator elevator) {this.elevator = elevator;}

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.elevator.moveToState(Elevator.ElevatorState.L3);
    }


}


