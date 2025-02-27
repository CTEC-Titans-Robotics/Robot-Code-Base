package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.Arm;
import frc8768.robot.subsystems.Elevator;

public class AuxiliaryOperator extends Operator {
    private final XboxController controller;
    private final Elevator elevator;
    private final Arm arm;

    private int release = 0;

    public AuxiliaryOperator(XboxController controller, Elevator elevator, Arm arm) {
        super("Auxiliary");
        this.controller = controller;
        this.elevator = elevator;
        this.arm = arm;
    }

    @Override
    public void run() {
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
        } else if (controller.getLeftStickButton()) {
            elevator.moveToState(Elevator.ElevatorState.ZERO);
            arm.moveToState(Arm.ArmState.ZERO);
        }

        if (controller.getLeftBumperButton()) {
            //align(); //TODO left align
        } else if (controller.getRightBumperButton()) {
            //align(); //TODO right align
        }

        if (controller.getLeftTriggerAxis() > 0.1 && elevator.isAtTarget()){
            arm.spinIntake(true);
            release = 1;
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
            arm.moveToState(Arm.ArmState.ZERO);
            elevator.moveToState(Elevator.ElevatorState.ZERO);
        }

        /* // TODO implement align function
    public void move(double xSpeed, double ySpeed, double rot) {
        swerve.drive(new Translation2d(xSpeed, ySpeed), rot, false, true, Constants.BOT_CENTER);
    }

    public void align() {
        if(controller.getYButton() && !reangle) {
            reangle = true;
        }

        if(reangle) {
            if (!MathUtil.isNear(0, yaw, 2)) {
                move(0, 0, MathUtil.clamp(-Math.toRadians(yaw) / 1.5, -0.5, 0.5));
            } else {
                reangle = false;
                move(0, 0, 0);
            }
        }

        // linear follow attempt;
        if(controller.getXButton() && !strafe) {
            strafe = true;
        }

        List<?> targets = vision.getTargets();
        if(strafe && !targets.isEmpty()) {
            PhotonTrackedTarget target = (PhotonTrackedTarget) targets.get(0);
            Transform3d transform = target.getBestCameraToTarget();
            double xMov = 0;
            double yMov = 0;
            double rotMov = 0;

            // X: Forward
            if(!MathUtil.isNear(32, transform.getMeasureX().in(Inches), 3)) {
                xMov = -MathUtil.clamp((32-transform.getMeasureX().in(Inches))/60, -0.5, 0.5);
            }

            // Y: Left
            double yTranslation = transform.getMeasureY().in(Inches);
            if(!MathUtil.isNear(0, yTranslation, 3)) {
                yMov = MathUtil.clamp(yTranslation/50, -0.5, 0.5);
            }

            if(!MathUtil.isNear(-180, transform.getRotation().getMeasureZ().in(Degree), 12)) {
                rotMov = MathUtil.clamp(-transform.getRotation().getMeasureZ().in(Degree)/75, -0.1, 0.1);
            }

            if(xMov == 0.0 && yMov == 0.0 && rotMov == 0.0) {
                move(0, 0, 0);
                strafe = false;
            } else {
                move(xMov, yMov, rotMov);
            }
        }
    }
    */
    }
}
