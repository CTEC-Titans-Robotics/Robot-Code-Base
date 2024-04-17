package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private Thread armThread;
    private XboxController controller;
    private final TalonFX leftExtend;
    private final TalonFX rightExtend;
    private final TalonSRX intake;
    private boolean isEmergencyStopped;

    public ArmSubsystem(int leftExtend, int rightExtend, int intake, boolean[] invertArray, XboxController controller) {
        this.armThread = new Thread(this::armLoop, "Arm Peripheral Thread");
        this.controller = controller;
        this.leftExtend = new TalonFX(leftExtend);
        this.rightExtend = new TalonFX(rightExtend);
        this.intake = new TalonSRX(intake);

        // Conf
        this.leftExtend.setNeutralMode(NeutralModeValue.Brake);
        this.rightExtend.setNeutralMode(NeutralModeValue.Brake);
        this.leftExtend.setInverted(invertArray[0]);
        this.rightExtend.setInverted(invertArray[1]);
        this.intake.setInverted(invertArray[2]);

        this.armThread.start();
    }

    public void emergencyStop() {
        isEmergencyStopped = true;
    }

    public void emergencyReset() {
        isEmergencyStopped = false;
    }

    public void spinIntake(double value) {
        intake.set(TalonSRXControlMode.PercentOutput, value);
    }

    public void stopIntake() {
        intake.set(TalonSRXControlMode.PercentOutput, 0);
    }

    // Copy-Pasted from Apollo Codebase, will clean later
    private void armLoop() {
        while (true) {
            if(controller.getXButton() && controller.getYButton() && controller.getBButton() && controller.getAButton()) {
                emergencyStop();
                this.leftExtend.set(0);
                this.rightExtend.set(0);
            }

            if(!isEmergencyStopped) {
                if (controller.getLeftBumperPressed() || controller.getAButtonPressed()) {
                    while (this.rightExtend.getPosition().getValue() * 0.17578152 < 150) {
                        this.leftExtend.set(-0.35);
                        this.rightExtend.set(0.35);
                    }
                } else if(this.rightExtend.getPosition().getValue() * 0.17578152 > 145) {
                    this.leftExtend.set(0.0);
                    this.rightExtend.set(0.0);
                }
                if (!controller.getLeftBumper() && !controller.getAButton() && (this.rightExtend.getPosition().getValue() * 0.17578152 > 25 || Math.abs(this.leftExtend.getPosition().getValue()) * 0.17578152 > 25)) {
                    while (this.rightExtend.getPosition().getValue() * 0.17578152 > 10) {
                        this.leftExtend.set(0.35);
                        this.rightExtend.set(-0.35);
                    }
                } else if(this.rightExtend.getPosition().getValue() * 0.17578152 < 0) {
                    this.leftExtend.set(0.01);
                    this.rightExtend.set(-0.01);
                }
            }
        }
    }
}
