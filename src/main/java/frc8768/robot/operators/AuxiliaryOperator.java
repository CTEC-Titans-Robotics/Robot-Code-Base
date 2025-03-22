package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class AuxiliaryOperator extends Operator {
    private final XboxController controller;
    private final Elevator elevator;
    private final Arm arm;

    public int release = 0;

    public AuxiliaryOperator(XboxController controller, Elevator elevator, Arm arm) {
        super("Auxiliary");
        this.controller = controller;
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    public void run() {
        /*
        if(controller.getRightStickButtonPressed() && elevator.state() == Elevator.ElevatorState.ZERO  ) {
            elevator.zeroElevator();
        }
        */

       if (controller.getPOV() == 180 && elevator.state() != Elevator.ElevatorState.L4) {
            elevator.moveToState(Elevator.ElevatorState.L1);
            arm.moveToState(Arm.ArmState.L1);
        } else if (controller.getPOV() == 0 && elevator.state() != Elevator.ElevatorState.L4) {
            elevator.moveToState(Elevator.ElevatorState.L2);
            arm.moveToState(Arm.ArmState.L2 );

           /* elevator.moveToState(Elevator.ElevatorState.L4);
            arm.moveToState(Arm.ArmState.CORAL );*/
       /* } else if (controller.getPOV() == 90) {
            elevator.moveToState(Elevator.ElevatorState.L1);
            arm.moveToState(Arm.ArmState.CORAL);

        */
        } else if (controller.getAButton()) {
            elevator.moveToState(Elevator.ElevatorState.L3);
            arm.moveToState(Arm.ArmState.L3);
       } else if (controller.getYButton()) {
            elevator.moveToState(Elevator.ElevatorState.L4);
            arm.moveToState(Arm.ArmState.L4);
        } else if (controller.getLeftStickButton()) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.ZERO);
        }


        //if (controller.getLeftTriggerAxis() > 0.1 && elevator.isAtTarget()){
       if (controller.getLeftTriggerAxis() > 0.1 ){
            arm.spinIntake(true);
            release = 1;
            //arm.moveToState(Arm.ArmState.HOLD);
            if(controller.getLeftTriggerAxis() <0.1){
                elevator.moveToState(Elevator.ElevatorState.ZERO);
                if(elevator.isAtTarget()){
                    arm.moveToState(Arm.ArmState.INTAKE);
                }
            }


        } else if (controller.getRightTriggerAxis() > 0.1 && elevator.state() == Elevator.ElevatorState.ZERO && elevator.isAtTarget()) {
            arm.spinIntake(false);
            arm.moveToState(Arm.ArmState.INTAKE);
            release = 1;
        } else {
            if(release == 1) {
                release = 2;
            }

            arm.stopIntake();
        }

        if(release >= 2) {
            release = 0;
            //arm.moveToState(Arm.ArmState.HOLD);
            elevator.moveToState(Elevator.ElevatorState.ZERO);
        }

    }
}
