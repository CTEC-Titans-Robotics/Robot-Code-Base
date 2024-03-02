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

    public AuxiliaryOperator(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        super("Auxiliary");

        this.arm = armSubsystem;
        this.intake = intakeSubsystem;
    }

    @Override
    public void run() {
        if(controller.getRightBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);
        } else if(controller.getLeftBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.AMP);
        } else {
            this.arm.releaseLock();
        }

        if(controller.getRightTriggerAxis() > Constants.controllerDeadband) {
            this.intake.beginStage(this.arm.currState == ArmSubsystem.ArmState.SPEAKER ?
                    IntakeSubsystem.IntakeStage.SPEAKER : IntakeSubsystem.IntakeStage.AMP);
        } else {
            this.intake.releaseLock();
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
