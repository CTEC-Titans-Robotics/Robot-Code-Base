package frc8768.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

/**
 * Subsystem for all things Tank related (Spark Max)
 */
public class TankSubsystemSpark implements Subsystem {
    /**
     * Left motors
     */
    private final CANSparkMax[] motorsLeft;

    /**
     * Right motors
     */
    private final CANSparkMax[] motorsRight;

    /**
     * Tank subsystem constructor.
     *
     * @param motorIdsLeft CAN IDs for left side motors.
     * @param motorIdsRight CAN IDs for right side motors.
     * @param motorsLeftInverted Left IDs -> inverted.
     * @param motorsRightInverted Right IDs -> inverted.
     * @param type Brushless or Brushed, see {@link com.revrobotics.CANSparkLowLevel.MotorType}
     */
    public TankSubsystemSpark(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted, CANSparkLowLevel.MotorType type) {
        motorsLeft = createSparkMax(motorIdsLeft, motorsLeftInverted, type);
        motorsRight = createSparkMax(motorIdsRight, motorsRightInverted, type);
    }

    /**
     * @param ids A list of CAN-IDs
     * @param inverted A list of inverted motors, index 0 of {@param ids} corresponds to index 0 of {@param inverted}
     * @param type Brushless or Brushed, see {@link com.revrobotics.CANSparkLowLevel.MotorType}
     * @return An array of Neo Motors.
     */
    private CANSparkMax[] createSparkMax(Set<Integer> ids, boolean[] inverted, CANSparkLowLevel.MotorType type) {
        CANSparkMax[] motors = new CANSparkMax[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[id] = new CANSparkMax(id, type);
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
        for(CANSparkMax motor : motorsLeft) {
            motor.set(translation2d.getX());
        }
        for(CANSparkMax motor : motorsRight) {
            motor.set(translation2d.getY());
        }
    }
}
