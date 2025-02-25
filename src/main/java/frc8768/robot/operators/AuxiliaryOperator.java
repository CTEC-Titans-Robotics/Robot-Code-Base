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
        if (controller.getPOV() == 180) {
            // TODO lvl 1
        } else if (controller.getPOV() == 0) {
            //TODO lvl 2
        } else if (controller.getAButton()) {
            //TODO lvl 3
        } else if (controller.getYButton()) {
            //TODO lvl 4
        }

        if (controller.getLeftBumperButton()) {
            //TODO left allign
        } else if (controller.getRightBumperButton()) {
            //TODO right allign
        }

        if (controller.getLeftTriggerAxis() > 0.1){
            arm.spinIntake(true);
        } else if (controller.getRightTriggerAxis() > 0.1) {
            arm.spinIntake(false);
        } else {
            arm.stop();
        }
    }
}
