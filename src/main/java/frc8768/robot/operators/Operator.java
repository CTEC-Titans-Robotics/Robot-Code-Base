package frc8768.robot.operators;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public interface Operator {
    String getName();
    int getControllerId();
    void configureBindings();

    default CommandXboxController getController() {
        return new CommandXboxController(getControllerId());
    }
}