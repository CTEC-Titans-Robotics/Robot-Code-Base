package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private CANSparkMax intakeMotor;
    private IntakeStates intakeState;
    private Timer timer;

    public IntakeSubsystem(int intakeMotorId) {
        intakeMotor = new CANSparkMax(intakeMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeState = IntakeStates.HELD;
        timer = new Timer();
    }

    public void tick() {
        switch(intakeState) {
            case INTAKE -> {
                if(timer.hasElapsed(0.25) && intakeMotor.getOutputCurrent() >= 80) {
                    intakeState = IntakeStates.HELD;
                    intakeMotor.set(-0.15);
                    timer.stop();
                    timer.reset();
                }
                if(timer.hasElapsed(7.5)) {
                    intakeState = IntakeStates.HELD;
                    intakeMotor.set(0);
                    timer.stop();
                    timer.reset();
                }
            }
            case EXPEL -> {
                if(timer.hasElapsed(0.5)) {
                    intakeMotor.set(0);
                    timer.stop();
                }
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
                intakeState = IntakeStates.EXPEL;
                timer.reset();
                timer.start();
                intakeMotor.set(0.5);
            }
        }
    }

    enum IntakeStates {
        HELD,
        INTAKE,
        EXPEL
    }
}
