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
            .idleMode(SparkBaseConfig.IdleMode.kCoast);
    private static final double angleOffset = 0; // FIXME
    private static final double upperBound = 0; // FIXME
    private static final double lowerBound = 0; // FIXME

    private final SparkFlex intakeMotor, pivotMotor;
    private final CANcoder absEncoder;
    private final Lock pivotLock = new ReentrantLock();
    private final Lock intakeLock = new ReentrantLock();


    public Arm() {
        pivotMotor = new SparkFlex(19, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new SparkFlex(20, SparkLowLevel.MotorType.kBrushless);
        absEncoder = new CANcoder(22, "auxiliary");

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

    private void forward() {
        if(pivotLock.tryLock()) {
            if (upperBound > getPosition()) {
                stop();
            } else {
                pivotMotor.set(
                        MathUtil.clamp(Math.abs(upperBound / getPosition()), 0, 0.3)
                );
            }
        }
    }

    private void backwards() {
        if(pivotLock.tryLock()) {
            if (lowerBound < getPosition()) {
                stop();
            } else {
                pivotMotor.set(
                        MathUtil.clamp(-Math.abs(lowerBound / getPosition()), -0.3, 0)
                );
            }
        }
    }

    public void stop() {
        try {
            pivotLock.unlock();
        } catch (Exception e) {
            // Ignore, some other thread has the lock
        } finally {
            pivotMotor.set(0);
        }
    }

    public void spinIntake(boolean outTake) {
        if(intakeLock.tryLock()) {
            intakeMotor.set(outTake ? 0.5 : -0.30);
        }
    }

    public void stopIntake() {
        try {
            intakeLock.unlock();
        } catch (Exception e) {
            // Ignore, some other thread has the lock
        } finally {
            intakeMotor.set(0);
        }
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("Arm position", getPosition());
        return map;
    }
}