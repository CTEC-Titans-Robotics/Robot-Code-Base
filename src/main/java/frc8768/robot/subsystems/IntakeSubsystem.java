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

        intakeMotorFollower.follow(intakeMotorLeader, true);
    }

    public void tick() {
        if(currState == IntakeState.INTAKE) {
            if(intakeMotorLeader.getOutputCurrent() > 45) {
                currState = IntakeState.HOLD;
            }
        }

        if(currState.dwellTime >= timer.get() && currState.dwellTime != -1) {
            currState = IntakeState.IDLE;
        }

        intakeMotorLeader.set(currState.speed);
    }

    public void run() {
        timer.reset();
        currState = currState.next == null
                ? IntakeState.IDLE : currState.next;
    }

    public enum IntakeState {
        IDLE(-1, 0, null),
        OUTTAKE(3, -0.4, IDLE),
        HOLD(-1, 0.2, OUTTAKE),
        INTAKE(5, 0.4, HOLD);

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
