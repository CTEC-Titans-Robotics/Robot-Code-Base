package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;

public class AuxiliaryOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public AuxiliaryOperator() {
        super("Auxiliary");

        this.arm = new ArmSubsystem();
        this.intake = new IntakeSubsystem();

        LogUtil.registerDashLogger(this.arm::dashboard);
        LogUtil.registerDashLogger(this.intake::dashboard);
    }

    @Override
    public void run() {
        this.intake.tick();

        if(controller.getLeftTriggerAxis() > Constants.controllerDeadband) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
            this.intake.setStage(IntakeSubsystem.IntakeStage.INTAKE);

        } else if(controller.getRightBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.AMP);

        } else if(controller.getLeftBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);

        } else if(!this.intake.isActive()) {
            // Don't let the drivers drive around with the arm down.
            // *we know how that went last time*
            this.arm.setDesiredState(ArmSubsystem.ArmState.IDLE);
        }

        if(controller.getRightTriggerAxis() > Constants.controllerDeadband) {
            this.intake.setStage(this.arm.currState == ArmSubsystem.ArmState.SPEAKER ?
                    IntakeSubsystem.IntakeStage.SPEAKER : IntakeSubsystem.IntakeStage.AMP);

        }

        // Emergency
        if(controller.getLeftStickButtonPressed() && controller.getRightStickButtonPressed()
                && controller.getAButtonPressed() && controller.getBButtonPressed()
                && controller.getXButtonPressed() && controller.getYButtonPressed()) {
            this.arm.stop();
            this.intake.stop();
        }
    }
}
