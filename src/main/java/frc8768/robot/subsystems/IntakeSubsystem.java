package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
        main = (TalonSRX[]) createFalcons(mainIds, 1, invertedMain);
        tip = (TalonFX[]) createFalcons(tipIds, 0, invertedTip);
    }

    private BaseTalon[] createFalcons(Set<Integer> ids, int type, boolean[] inverted) {
        BaseTalon[] motors = new BaseTalon[ids.size()];
        int i = 0;
        for(int id : ids) {
            if(type == 0) {
                motors[id] = new TalonFX(id);
            } else {
                motors[id] = new TalonSRX(id);
            }
            motors[id].setInverted(inverted[i]);
            i++;
        }
        return motors;
    }

    public void spinMain(double value) {
        for(TalonSRX motor : main) {
            motor.set(ControlMode.PercentOutput, value);
        }
    }

    public void spinTip(double front, double back) {
        tip[0].set(ControlMode.PercentOutput, front);
        tip[1].set(TalonFXControlMode.PercentOutput, back);
    }
}
