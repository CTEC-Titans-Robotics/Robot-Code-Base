package frc8768.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

/**
 * Subsystem for all things Tank related (Falcon 500s, Krakens)
 */
public class TankSubsystemFalcon implements Subsystem {
    /**
     * Left motors
     */
    private final Talon[] motorsLeft;

    /**
     * Right motors
     */
    private final Talon[] motorsRight;

    /**
     * Tank subsystem constructor.
     *
     * @param motorIdsLeft CAN IDs for left side motors.
     * @param motorIdsRight CAN IDs for right side motors.
     * @param motorsLeftInverted Left IDs -> inverted.
     * @param motorsRightInverted Right IDs -> inverted.
     */
    public TankSubsystemFalcon(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted) {
        motorsLeft = createTalon(motorIdsLeft, motorsLeftInverted);
        motorsRight = createTalon(motorIdsRight, motorsRightInverted);
    }

    /**
     * @param ids A list of CAN-IDs
     * @param inverted A list of inverted motors, index 0 of {@param ids} corresponds to index 0 of {@param inverted}
     * @return An array of Talon Motors.
     */
    private Talon[] createTalon(Set<Integer> ids, boolean[] inverted) {
        Talon[] motors = new Talon[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[i] = new Talon(id);
            motors[i].setInverted(inverted[i]);
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
        for(Talon motor : motorsLeft) {
            motor.set(translation2d.getX());
        }
        for(Talon motor : motorsRight) {
            motor.set(translation2d.getY());
        }
    }
}
