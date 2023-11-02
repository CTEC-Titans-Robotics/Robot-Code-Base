package frc8768.robot.operators;

import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.util.Constants;

public class IntakeOperator extends Operator {
    private final XboxController controller = new XboxController(Constants.coDriverControllerId);

    public IntakeOperator() {
        super("Intake");
    }

    @Override
    public void run() {

    }
}
