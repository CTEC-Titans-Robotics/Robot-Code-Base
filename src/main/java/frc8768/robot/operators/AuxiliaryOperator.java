package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;

public class AuxiliaryOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;

    public AuxiliaryOperator() {
        super("Auxiliary");

        this.arm = new ArmSubsystem();
        this.intake = new IntakeSubsystem();
    }

    @Override
    public void run() {
        this.intake.tick();

        if(controller.getAButtonPressed()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.INTAKE);
            this.intake.setStage(IntakeSubsystem.IntakeStage.NOTE_PICKUP);

        } else if(controller.getXButtonPressed()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.AMP);

        } else if(controller.getYButtonPressed()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);

        } else if(controller.getBButtonPressed()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.IDLE);
            this.intake.setStage(IntakeSubsystem.IntakeStage.IDLE);
        }

        if(controller.getRightBumperPressed()) {
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
