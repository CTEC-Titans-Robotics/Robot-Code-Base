package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;

public class PeripheralOperator extends Operator {
    private final XboxController controller = new XboxController(Constants.coDriverControllerId);
    private final ArmSubsystem armSubsystem = new ArmSubsystem(15);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(16, 17);

    public PeripheralOperator() {
        super("Intake");
    }

    @Override
    public void run() {
        armSubsystem.tick();
        intakeSubsystem.tick();

        if(controller.getBButtonPressed()) {
            armSubsystem.up();
        }
        if(controller.getXButtonPressed()) {
            armSubsystem.down();
        }

        if(controller.getRightBumperPressed()) {
            intakeSubsystem.run();
        }
    }
}
