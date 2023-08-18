package frc8768.robot.operators;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;

import java.util.Set;

public class PeripheralOperator extends Operator{
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    public PeripheralOperator() {
        super("Peripheral", controller);

        arm = new ArmSubsystem(0, 0 ,0, controller);
        intake = new IntakeSubsystem(Set.of(0, 0), Set.of(0, 0), new boolean[] { false, false }, new boolean[] { false, false });
    }

    @Override
    public void run() {
        if (controller.getRightTriggerAxis() > 0.1) {
            intake.spinTip(-0.50, 0.10);
        } else {
            intake.spinTip(0, 0);
        }

        if(controller.getRightBumper()) {
            intake.spinMain(0.5);
            arm.spinIntake(0.35);
        } else if(controller.getAButton()) {
            intake.spinMain(-0.5);
            arm.spinIntake(-0.35);
        } else {
            intake.spinMain(0);
            arm.spinIntake(0);
        }
    }
}
