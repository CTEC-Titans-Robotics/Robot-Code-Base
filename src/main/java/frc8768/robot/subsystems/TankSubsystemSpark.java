package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

// Big ol' fuck you from Falcons, they don't implement IMotorController
public class TankSubsystemSpark implements Subsystem {
    private final CANSparkMax[] motorsLeft;
    private final CANSparkMax[] motorsRight;

    public TankSubsystemSpark(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted, CANSparkMaxLowLevel.MotorType type) {
        motorsLeft = createSparkMax(motorIdsLeft, motorsLeftInverted, type);
        motorsRight = createSparkMax(motorIdsRight, motorsRightInverted, type);
    }

    private CANSparkMax[] createSparkMax(Set<Integer> ids, boolean[] inverted, CANSparkMaxLowLevel.MotorType type) {
        CANSparkMax[] motors = new CANSparkMax[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[id] = new CANSparkMax(id, type);
            motors[id].setInverted(inverted[i]);
            i++;
        }
        return motors;
    }

    public void drive(Translation2d translation2d) {
        for(CANSparkMax motor : motorsLeft) {
            motor.set(translation2d.getX());
        }
        for(CANSparkMax motor : motorsRight) {
            motor.set(translation2d.getY());
        }
    }
}
