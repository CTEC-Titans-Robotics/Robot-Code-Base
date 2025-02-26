package frc8768.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Arm implements Subsystem {
    private static final SparkBaseConfig PIVOT_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final SparkBaseConfig INTAKE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final double angleOffset = -26.015625;

    private ArmState currState = ArmState.ZERO;
    private final SparkFlex intakeMotor, pivotMotor;
    private final CANcoder absEncoder;
    private final Lock intakeLock = new ReentrantLock();


    public Arm() {
        pivotMotor = new SparkFlex(19, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new SparkFlex(20, SparkLowLevel.MotorType.kBrushless);
        absEncoder = new CANcoder(22);

        pivotMotor.configure(PIVOT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        intakeMotor.configure(INTAKE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        LogUtil.registerDashLogger(this::dashLog);
    }

    /**
     * @return Degrees
     */
    private double getPosition() {
        return absEncoder.getPosition().getValue().in(Units.Degree) - angleOffset;
    }

    public void tick() {
        if (!MathUtil.isNear(currState.targetPosition, getPosition(), 5) && currState != ArmState.ZERO) {
            if (currState.targetPosition > getPosition()) {
                pivotMotor.set(-0.15);
            } else {
                pivotMotor.set(0.15);
            }
        } else {
            stop();
        }
    }

    public void stop() {
        pivotMotor.set(-0.08);
    }

    public void spinIntake(boolean outTake) {
        if(intakeLock.tryLock()) {
            intakeMotor.set(outTake ? -0.1 : 0.1);
        }
    }

    public void stopIntake() {
        try {
            intakeLock.unlock();
        } catch (Exception e) {
            // Ignore, some other thread has the lock
        } finally {
            intakeMotor.set(0.02);
        }
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("Arm position", getPosition());
        return map;
    }

    public void moveToState(ArmState state) {
        currState = state;
    }

    public enum ArmState {
        ZERO(0),
        L1(22), // FIXME
        L2(11),
        L3(33), // FIXME
        L4(125),
        INTAKE(-155);

        final double targetPosition;

        ArmState(double angleDegrees) {
            this.targetPosition = angleDegrees;
        }
    }
}