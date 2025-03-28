package frc8768.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class Reset extends Command{

    private final Arm arm;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    public Reset(Elevator elevator, Arm arm) {
        this.arm = arm;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        arm.stopIntake();
        elevator.moveToState(Elevator.ElevatorState.ZERO);
        if(elevator.isAtTarget()){
            arm.moveToState(Arm.ArmState.INTAKE);
        }

        if(arm.isAtRotation()){
            arm.spinIntake(false);
        }
    }
}
