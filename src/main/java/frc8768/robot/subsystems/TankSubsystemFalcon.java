package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

// Big ol' fuck you from Falcons, they don't implement IMotorController
public class TankSubsystemFalcon implements Subsystem {
    private final TalonFX[] motorsLeft;
    private final TalonFX[] motorsRight;

    public TankSubsystemFalcon(Set<Integer> motorIdsLeft, Set<Integer> motorIdsRight, boolean[] motorsLeftInverted, boolean[] motorsRightInverted) {
        motorsLeft = createFalconFX(motorIdsLeft, motorsLeftInverted);
        motorsRight = createFalconFX(motorIdsRight, motorsRightInverted);
    }

    private TalonFX[] createFalconFX(Set<Integer> ids, boolean[] inverted) {
        TalonFX[] motors = new TalonFX[ids.size()];
        int i = 0;
        for(int id : ids) {
            motors[i] = new TalonFX(id);
            motors[i].configFactoryDefault();
            motors[i].setNeutralMode(NeutralMode.Brake);
            motors[i].setInverted(inverted[i]);
            i++;
        }
        return motors;
    }

    public void drive(Translation2d translation2d) {
        for(TalonFX motor : motorsLeft) {
            motor.set(ControlMode.PercentOutput, translation2d.getX());
        }
        for(TalonFX motor : motorsRight) {
            motor.set(ControlMode.PercentOutput, translation2d.getY());
        }
    }
}
