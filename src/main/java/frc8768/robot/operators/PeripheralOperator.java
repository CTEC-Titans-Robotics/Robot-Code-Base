package frc8768.robot.operators;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmExtensionSubsystem;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;

public class PeripheralOperator extends Operator {
    private static XboxController controller = new XboxController(Constants.coDriverControllerId);
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ArmExtensionSubsystem extensionSubsystem;

    private boolean hasZeroed = false;

    public PeripheralOperator() {
        super("Peripheral");

        armSubsystem = new ArmSubsystem(16, 15, 19, 1);
        extensionSubsystem = new ArmExtensionSubsystem(17, 9);
        intakeSubsystem = new IntakeSubsystem(18);
    }

    @Override
    public void run() {
        armSubsystem.tick();
        extensionSubsystem.tick();
        intakeSubsystem.tick();

        if(!hasZeroed) {
            hasZeroed = true;
            //armSubsystem.zeroArm();
            extensionSubsystem.zeroExtension();
        }

        if(controller.getPOV() == 180) {
            armSubsystem.magicButton(-62);
        }

        if(controller.getPOV() == 0) {
            armSubsystem.magicButton(-19);
        }

        if(controller.getRightBumperPressed()) {
            intakeSubsystem.spin();
        }
        if(controller.getYButton()) {
            extensionSubsystem.zeroExtension();
        }

        if(controller.getRightY() > 0.1) {
            armSubsystem.moveArm(0.325);
        } else if(controller.getRightY() < -0.1) {
            armSubsystem.moveArm(-0.25);
        } else {
            armSubsystem.stopGearbox();
        }

        if(controller.getLeftY() > 0.1) {
            extensionSubsystem.moveArmExtension(-0.1);
        } else if(controller.getLeftY() < -0.1) {
            extensionSubsystem.moveArmExtension(0.1);
        } else {
            extensionSubsystem.stopExtension();
        }
    }
}
