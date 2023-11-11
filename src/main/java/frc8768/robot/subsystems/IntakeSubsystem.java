package frc8768.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private final CANSparkMax intakeMotorLeader;
    private final CANSparkMax intakeMotorFollower;
    private IntakeState currState = IntakeState.IDLE;
    private Timer timer = new Timer();

    public IntakeSubsystem(int leaderId, int followerId) {
        intakeMotorLeader = new CANSparkMax(leaderId, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotorFollower = new CANSparkMax(followerId, CANSparkMaxLowLevel.MotorType.kBrushless);

        // Conf
        intakeMotorLeader.restoreFactoryDefaults();
        intakeMotorLeader.setSmartCurrentLimit(25);
        intakeMotorLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeMotorLeader.burnFlash();

        intakeMotorFollower.restoreFactoryDefaults();
        intakeMotorFollower.setSmartCurrentLimit(30);
        intakeMotorFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        // intakeMotorFollower.setInverted(true);
        intakeMotorFollower.burnFlash();

        timer.start();
    }

    public void tick() {
        if(currState == IntakeState.INTAKE) {
            if(intakeMotorLeader.getOutputCurrent() > 30) {
                currState = IntakeState.HOLD;
            }
        }

        if(timer.get() >= currState.dwellTime && currState.dwellTime != -1) {
            currState = IntakeState.IDLE;
        }

        intakeMotorLeader.set(currState.speed);
        intakeMotorFollower.set(-currState.speed);
    }

    public void run() {
        timer.reset();
        currState = currState.next == null
                ? IntakeState.INTAKE : currState.next;
    }

    public void outtake() {
        intakeMotorLeader.set(-0.4);
        timer.reset();
        while(timer.get() < 2);
        intakeMotorLeader.set(0);
    }

    public enum IntakeState {
        IDLE(-1, 0, null),
        OUTTAKE(1.5, -0.5, IDLE),
        HOLD(-1, 0.05, OUTTAKE),
        INTAKE(5, 0.35, HOLD);

        double dwellTime;
        double speed;
        IntakeState next;
        IntakeState(double dwellTime, double speed, IntakeState nextState) {
            this.dwellTime = dwellTime;
            this.speed = speed;
            this.next = nextState;
        }
    }
}
