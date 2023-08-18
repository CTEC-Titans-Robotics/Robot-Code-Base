package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

// Big ol' fuck you from Falcons, they don't implement IMotorController
public class TankSubsystemFalcon implements Subsystem {
    private final BaseTalon[] motorsLeft;
    private final BaseTalon[] motorsRight;

    public TankSubsystemFalcon(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted, int type) {
        motorsLeft = createFalcons(motorIdsLeft, type, motorsLeftInverted);
        motorsRight = createFalcons(motorIdsRight, type, motorsRightInverted);
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

    public void drive(Translation2d translation2d) {
        for(BaseTalon motor : motorsLeft) {
            motor.set(ControlMode.PercentOutput, translation2d.getX());
        }
        for(BaseTalon motor : motorsRight) {
            motor.set(ControlMode.PercentOutput, translation2d.getY());
        }
    }
}
