package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class IntakeSubsystem implements Subsystem {
    private final TalonSRX[] main;
    private final TalonFX[] tip;

    public IntakeSubsystem(Set<Integer> mainIds, Set<Integer> tipIds, boolean[] invertedMain, boolean[] invertedTip) {
        main = createTalonSRX(mainIds, invertedMain);
        tip = createTalonFX(tipIds, invertedTip);
    }

    private TalonFX[] createTalonFX(Set<Integer> ids, boolean[] inverted) {
        TalonFX[] motors = new TalonFX[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[i] = new TalonFX(id);
            motors[i].configFactoryDefault();
            motors[i].setNeutralMode(NeutralMode.Coast);
            motors[i].setInverted(inverted[i]);
            i++;
        }
        return motors;
    }

    private TalonSRX[] createTalonSRX(Set<Integer> ids, boolean[] inverted) {
        TalonSRX[] motors = new TalonSRX[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[i] = new TalonSRX(id);
            motors[i].configFactoryDefault();
            motors[i].setNeutralMode(NeutralMode.Coast);
            motors[i].setInverted(inverted[i]);
            i++;
        }
        return motors;
    }

    public void spinMain(double front, double back) {
        main[0].set(ControlMode.PercentOutput, front);
        main[1].set(ControlMode.PercentOutput, back);
    }

    public void spinTip(double front, double back) {
        tip[0].set(ControlMode.PercentOutput, front);
        tip[1].set(TalonFXControlMode.PercentOutput, back);
    }

    public void stopTip() {
        tip[0].set(ControlMode.PercentOutput, 0);
        tip[1].set(TalonFXControlMode.PercentOutput, 0);
    }
}
