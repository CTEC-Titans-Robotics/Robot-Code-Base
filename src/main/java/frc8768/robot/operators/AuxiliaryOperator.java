package frc8768.robot.operators;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc8768.robot.subsystems.ArmSubsystem;
import frc8768.robot.subsystems.IntakeSubsystem;
import frc8768.robot.util.Constants;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.LimelightVision;
import frc8768.visionlib.PhotonVision;

import java.util.List;

public class AuxiliaryOperator extends Operator {
    private static final XboxController controller = new XboxController(Constants.coDriverControllerId);

    private final ArmSubsystem arm;
    private final CANdle caNdle;
    private final IntakeSubsystem intake;
    private final PhotonVision vision;

    public AuxiliaryOperator(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
        super("Auxiliary");

        this.arm = armSubsystem;
        this.intake = intakeSubsystem;

        // LEDs
        this.caNdle = new CANdle(36);
        this.caNdle.configLEDType(CANdle.LEDStripType.GRB);
        this.caNdle.configV5Enabled(true);
        this.caNdle.setLEDs(0, 0, 255);

        this.vision = new PhotonVision("limelight-left");

        LogUtil.registerDashLogger(this.arm::dashboard);
        LogUtil.registerDashLogger(this.intake::dashboard);
    }

    @Override
    public void run() {
        this.intake.tick();

        double distance =
                this.vision.getDistanceToTarget(30, 0, 57.13, false);

        if(controller.getRightBumper()) {
            this.arm.setDesiredState(ArmSubsystem.ArmState.SPEAKER);
        } else if(controller.getLeftBumper()) {
            if(distance != -1 && Constants.SPEAKER_IDS.contains(this.vision.getTargetID())) {
                this.caNdle.setLEDs(0, 255, 0);
                this.arm.overrideAngle = MathUtil.clamp(Math.pow(distance, 0.675) + 30, 2, 85);
            } else {
                this.caNdle.setLEDs(255, 0, 0);
                this.arm.overrideAngle = -1;
            }
            this.arm.setDesiredState(ArmSubsystem.ArmState.AMP);
        } else {
            this.arm.releaseLock();
        }

        if(controller.getRightTriggerAxis() > Constants.controllerDeadband) {
            if(distance != -1 && Constants.SPEAKER_IDS.contains(this.vision.getTargetID())) {
                this.intake.overrideShootSpeed = MathUtil.clamp((0.005 * distance) + 0.18, 0, 1);
            } else {
                this.intake.overrideShootSpeed = -1;
            }

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
