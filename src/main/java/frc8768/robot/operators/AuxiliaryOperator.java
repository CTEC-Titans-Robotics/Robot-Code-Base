package frc8768.robot.operators;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.PhotonVision;

import java.util.Map;

public class AuxiliaryOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    private final ArmSubsystem arm;
    private final CANdle caNdle;
    private final IntakeSubsystem intake;
    private PhotonVision vision;
    private double readDistance = -1;

    public AuxiliaryOperator(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, PhotonVision vision) {
        super("Auxiliary");

        this.arm = armSubsystem;
        this.intake = intakeSubsystem;

        // LEDs
        this.caNdle = new CANdle(21);
        this.caNdle.configLEDType(CANdle.LEDStripType.GRB);
        this.caNdle.configV5Enabled(true);
        this.caNdle.setLEDs(0, 0, 255);

        this.vision = vision;

        LogUtil.registerDashLogger(this::log);
    }

    public void log(Map<String, String> map) {
        map.put("Distance", String.valueOf(this.readDistance));
    }

    @Override
    public void run() {
        this.intake.tick();

        double tempDistance = calcDist(this.vision.getDistanceToTarget(35, 21.5, 57.13, false));
        this.readDistance = /*controller.getAButton() ?*/ -1/* : 0.00904539 * Math.pow(tempDistance, 2) + 0.520543*tempDistance+1.52968*/;
        if(controller.getRightBumper()) {
            // if(this.readDistance != -1 && Constants.SPEAKER_IDS.contains(this.vision.getTargetID())) {
            //     this.caNdle.setLEDs(0, 255, 0);
            //     this.arm.overrideAngle = MathUtil.clamp((-0.00324675 * Math.pow(this.readDistance, 2) + 0.717532 * this.readDistance + 10.3766), 1, 97);
            // }
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);
        } else if(controller.getLeftBumper()) {
            this.arm.overrideAngle = 42;
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);
        } else {
            this.arm.releaseLock();
            this.caNdle.setLEDs(255, 0, 0);
            this.arm.overrideAngle = -1;
        }

        if(controller.getRightTriggerAxis() > Constants.controllerDeadband) {
            // if(this.readDistance != -1 && Constants.SPEAKER_IDS.contains(this.vision.getTargetID())) {
            //     this.intake.overrideShootSpeed = MathUtil.clamp((-0.0000324675 * Math.pow(this.readDistance, 2)
            //             + 0.00967532 * this.readDistance - 0.126234), 0, 1);
            //     this.intake.overrideHoldSpeed = MathUtil.clamp((
            //                     (-0.0000450938 * Math.pow(this.readDistance, 2) + 0.013438 * this.readDistance) - 0.175325),
            //             0, 1);
            // }

            this.intake.beginStage(this.arm.currState == ArmSubsystem.ArmState.SPEAKER ?
                    IntakeSubsystem.IntakeStage.SPEAKER : IntakeSubsystem.IntakeStage.AMP);
        } else {
            this.intake.releaseLock();
            this.intake.overrideShootSpeed = -1;
            this.intake.overrideHoldSpeed = -1;
        }

        // Emergency
        if(controller.getLeftStickButtonPressed() && controller.getRightStickButtonPressed()
                && controller.getAButtonPressed() && controller.getBButtonPressed()
                && controller.getXButtonPressed() && controller.getYButtonPressed()) {
            this.arm.stop();
            this.intake.stop();
        }
    }

    private double calcDist(double input) {
        return -0.00325*Math.pow(input, 2)+0.717*input+10.4;
    }
}
