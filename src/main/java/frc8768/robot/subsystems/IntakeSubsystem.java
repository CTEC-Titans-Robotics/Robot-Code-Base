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
        intakeState = IntakeStates.EXPEL;
        timer = new Timer();
    }

    public void tick() {
        if (Objects.requireNonNull(intakeState) == IntakeStates.INTAKE) {
            if (timer.hasElapsed(0.25) && intakeMotor.getOutputCurrent() >= 80) {
                intakeState = IntakeStates.HELD;
                intakeMotor.set(-0.15);
                timer.stop();
                timer.reset();
            }
            if (timer.hasElapsed(7.5)) {
                intakeState = IntakeStates.EXPEL;
                intakeMotor.set(0);
                timer.stop();
                timer.reset();
            }
        }
    }

    public void spin() {
        switch (intakeState) {
            case EXPEL -> {
                intakeMotor.set(-0.75);
                timer.reset();
                timer.start();
                intakeState = IntakeStates.INTAKE;
            }
            case HELD -> {
                intakeMotor.set(0.5);
                timer.reset();
                timer.start();
                intakeState = IntakeStates.EXPEL;
            }
        }
    }

    enum IntakeStates {
        HELD,
        INTAKE,
        EXPEL
    }
}
