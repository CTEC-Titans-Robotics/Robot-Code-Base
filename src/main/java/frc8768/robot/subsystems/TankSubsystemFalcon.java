package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

/**
 * Subsystem for all things Tank related (Falcons)
 */
public class TankSubsystemFalcon implements Subsystem {
    /**
     * Left motors
     */
    private final BaseTalon[] motorsLeft;

    /**
     * Right motors
     */
    private final BaseTalon[] motorsRight;

    /**
     * Tank subsystem constructor.
     *
     * @param motorIdsLeft CAN IDs for left side motors.
     * @param motorIdsRight CAN IDs for right side motors.
     * @param motorsLeftInverted Left IDs -> inverted.
     * @param motorsRightInverted Right IDs -> inverted.
     * @param type Brushless or Brushed, 0 for TalonFX or 1 for TalonSRX
     */
    public TankSubsystemFalcon(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted, int type) {
        motorsLeft = createFalcons(motorIdsLeft, type, motorsLeftInverted);
        motorsRight = createFalcons(motorIdsRight, type, motorsRightInverted);
    }

    /**
     * @param ids A list of CAN-IDs
     * @param inverted A list of inverted motors, index 0 of {@param ids} corresponds to index 0 of {@param inverted}
     * @param type Brushless or Brushed, see {@link com.revrobotics.CANSparkMaxLowLevel.MotorType}
     * @return An array of Neo Motors.
     */
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

    /**
     * Drive the subsystem.
     *
     * @param translation2d X = Left, Y = Right.
     */
    public void drive(Translation2d translation2d) {
        for(BaseTalon motor : motorsLeft) {
            motor.set(ControlMode.PercentOutput, translation2d.getX());
        }
        for(BaseTalon motor : motorsRight) {
            motor.set(ControlMode.PercentOutput, translation2d.getY());
        }
    }
}
