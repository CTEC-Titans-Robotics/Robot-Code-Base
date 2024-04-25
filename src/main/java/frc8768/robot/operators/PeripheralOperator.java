package frc8768.robot.operators;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.Robot;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.Limelight;

import java.util.Set;

public class PeripheralOperator extends Operator{
    private final ArmSubsystem arm;
    private final IntakeSubsystem intake;
    private Limelight limelight;
    private final Timer shootTimer = new Timer();
    private final boolean isLimelightEnabled = false;

    private ShotMode currMode = ShotMode.CR_TOP;
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    public PeripheralOperator() {
        super("Peripheral");

        if(isLimelightEnabled) {
            limelight = new Limelight();
            limelight.init();
        }
        arm = new ArmSubsystem(13, 3, 8, new boolean[] { false, false, false }, controller);
        intake = new IntakeSubsystem(Set.of(6, 7), Set.of(10, 9), new boolean[] { true, true }, new boolean[] { false, true });
    }

    @Override
    public void run() {
        if(!Robot.isRobotTeleop()) {
            arm.emergencyReset();
            return;
        }

        if(isLimelightEnabled) {
            limelight.tick();
        }

        if(controller.getRightBumper()) {
            intake.spinMain(-0.5, 0.5);
        } else if(controller.getLeftBumper()) {
            intake.spinMain(-0.5, 0.5);
            intake.spinTip(0.2, -0.2);
            arm.spinIntake(0.35);
        } else if(controller.getAButton()) {
            intake.spinMain(0.5, -0.5);
            arm.spinIntake(-0.35);
        } else {
            intake.spinMain(0, 0);
            if(controller.getRightTriggerAxis() <= 0.5) {
                intake.stopTip();
            }
            arm.spinIntake(0);
        }

        if (controller.getPOV() >= 0 && controller.getPOV() < 90) { // Upper hoop close range
            currMode = ShotMode.CR_TOP;
        }
        if (controller.getPOV() >= 90 && controller.getPOV() < 180) { // upper hoop mid range
            currMode = ShotMode.MR_TOP;
        }
        if (controller.getPOV() >= 270 && controller.getPOV() < 360 ) { // upper hoop long range
            currMode = ShotMode.LR_TOP;
        }
        if (controller.getPOV() >= 180 && controller.getPOV() < 270 && !isLimelightEnabled) { // lower hoop close range
            currMode = ShotMode.LIMELIGHT;
        }

        if (controller.getRightTriggerAxis() > 0.5) {
            switch (currMode) {
                case LR_TOP -> intake.spinTip(-0.56, 0.08);
                case MR_TOP -> intake.spinTip(-0.50, 0.08);
                case CR_TOP -> intake.spinTip(-0.50, 0.10);
                case LIMELIGHT -> {
                    double distance = limelight.getDistance();
                    if (distance > 300D/12D){
                        distance = 10;
                    } else if (distance < -1){
                        distance = 10;
                    }
                    intake.spinTip(0.0149*distance+.3428, 0.0027*distance+.0605);
                }
            }
        } else {
            if(!controller.getLeftBumper()) {
                intake.stopTip();
            }
            shootTimer.reset();
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    enum ShotMode {
        LR_TOP,
        MR_TOP,
        CR_TOP,
        LIMELIGHT
    }
}
