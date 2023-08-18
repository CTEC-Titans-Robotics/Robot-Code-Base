package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArmSubsystem implements Subsystem {
    private Thread armThread;
    private XboxController controller;
    private TalonFX leftExtend;
    private TalonFX rightExtend;
    private TalonSRX intake;
    private boolean isEmergencyStopped;

    public ArmSubsystem(int leftExtend, int rightExtend, int intake, XboxController controller) {
        this.armThread = new Thread(this::armLoop, "Arm Peripheral Thread");
        this.controller = controller;
        this.leftExtend = new TalonFX(leftExtend);
        this.rightExtend = new TalonFX(rightExtend);
        this.intake = new TalonSRX(intake);

        // Conf
        this.leftExtend.setNeutralMode(NeutralMode.Brake);
        this.rightExtend.setNeutralMode(NeutralMode.Brake);

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
                isEmergencyStopped = true;
                this.leftExtend.set(TalonFXControlMode.PercentOutput, 0);
                this.rightExtend.set(TalonFXControlMode.PercentOutput, 0);
            }

            if(!isEmergencyStopped) {
                if (controller.getLeftBumperPressed() || controller.getAButtonPressed()) {
                    while (this.rightExtend.getSelectedSensorPosition() * 0.17578152 < 150) {
                        this.leftExtend.set(TalonFXControlMode.PercentOutput, -0.35);
                        this.rightExtend.set(TalonFXControlMode.PercentOutput, 0.35);
                    }
                } else if(this.rightExtend.getSelectedSensorPosition()*0.17578152 > 145) {
                    this.leftExtend.set(TalonFXControlMode.PercentOutput, 0.0);
                    this.rightExtend.set(TalonFXControlMode.PercentOutput, 0.0);
                }
                if (!controller.getLeftBumper() && !controller.getAButton() && (this.rightExtend.getSelectedSensorPosition() * 0.17578152 > 25 || Math.abs(this.leftExtend.getSelectedSensorPosition()) * 0.17578152 > 25)) {
                    while (this.rightExtend.getSelectedSensorPosition() * 0.17578152 > 10) {
                        this.leftExtend.set(TalonFXControlMode.PercentOutput, 0.35);
                        this.rightExtend.set(TalonFXControlMode.PercentOutput, -0.35);
                    }
                } else if(this.rightExtend.getSelectedSensorPosition()*0.17578152 < 0) {
                    this.leftExtend.set(TalonFXControlMode.PercentOutput, 0.01);
                    this.rightExtend.set(TalonFXControlMode.PercentOutput, -0.01);
                }
            }
        }
    }
}
