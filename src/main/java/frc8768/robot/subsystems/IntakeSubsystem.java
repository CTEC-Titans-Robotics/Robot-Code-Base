package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Objects;

public class IntakeSubsystem implements Subsystem {
    private final CANSparkMax intakeMotor;
    private final Timer timer;
    private IntakeStates intakeState;

    public IntakeSubsystem(int intakeMotorId) {
        intakeMotor = new CANSparkMax(intakeMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeState = IntakeStates.IDLE;
        timer = new Timer();
    }

    private void stopAndReset() {
        intakeMotor.set(0);
        timer.stop();
        timer.reset();
    }

    public void tick() {
        switch (intakeState) {
            case INTAKE -> {
                if(timer.hasElapsed(6.5)) {
                    intakeState = IntakeStates.IDLE;
                    stopAndReset();
                }
                if(intakeMotor.getOutputCurrent() > 60) {
                    intakeState = IntakeStates.HELD;
                    intakeMotor.set(-0.1);
                    timer.reset();
                }
            }

            case EXPEL -> {
                if(timer.hasElapsed(2)) {
                    intakeState = IntakeStates.IDLE;
                    stopAndReset();
                }
            }
        }
    }

    public void spin() {
        switch (intakeState) {
            case IDLE -> {
                intakeState = IntakeStates.INTAKE;
                intakeMotor.set(-0.5);
                timer.start();
            }

            case HELD -> {
                intakeState = IntakeStates.EXPEL;
                intakeMotor.set(0.5);
                timer.reset();
            }
        }
    }

    enum IntakeStates {
        HELD,
        INTAKE,
        EXPEL,
        IDLE
    }
}
