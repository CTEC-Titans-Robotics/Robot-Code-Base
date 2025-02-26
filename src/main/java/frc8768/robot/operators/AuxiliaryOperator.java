package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class AuxiliaryOperator extends Operator {
    private final XboxController controller;
    private final Elevator elevator;
    private final Arm arm;

    public AuxiliaryOperator(XboxController controller, Elevator elevator, Arm arm) {
        super("Auxiliary");
        this.controller = controller;
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    public void run() {
        /*
        if(controller.getRightStickButtonPressed()) {
            elevator.zeroElevator();
        }

        if (controller.getPOV() == 180) {
            elevator.moveToState(Elevator.ElevatorState.L1);
            arm.moveToState(Arm.ArmState.L1);
        } else if (controller.getPOV() == 0) {
            elevator.moveToState(Elevator.ElevatorState.L2);
            arm.moveToState(Arm.ArmState.L2);
        } else if (controller.getAButton()) {
            elevator.moveToState(Elevator.ElevatorState.L3);
            arm.moveToState(Arm.ArmState.L3);
        } else if (controller.getYButton()) {
            elevator.moveToState(Elevator.ElevatorState.L4);
            arm.moveToState(Arm.ArmState.L4);
        } else if(controller.getPOV() == 90) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.ZERO);
        }

        if (controller.getLeftBumperButton()) {
            //TODO left align
        } else if (controller.getRightBumperButton()) {
            //TODO right align
        }

        if (controller.getLeftTriggerAxis() > 0.1){
            arm.spinIntake(true);
        } else if (controller.getRightTriggerAxis() > 0.1 && elevator.state() == Elevator.ElevatorState.ZERO) {
            arm.spinIntake(false);
            arm.moveToState(Arm.ArmState.INTAKE);
        } else {
            arm.stopIntake();
        }
         */
    }
}
