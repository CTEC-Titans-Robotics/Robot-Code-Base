package frc8768.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Elevator {
    private static final SparkBaseConfig ELEVATOR_BASE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkFlex elevatorMotor2, elevatorMotor1;
    private final Lock moveLock = new ReentrantLock();

    public Elevator() {
        elevatorMotor1 = new SparkFlex(17, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkFlex(18, SparkLowLevel.MotorType.kBrushless);

        elevatorMotor1.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG).inverted(true), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorMotor2.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG).follow(elevatorMotor1, true),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    /**
     * Up is positive, down is negative
     */
    public void up() {
        if(moveLock.tryLock()) {
            elevatorMotor1.set(0.1);
        }
    }

    public void down() {
        if(moveLock.tryLock()) {
            elevatorMotor1.set(-0.05);
        }
    }

    public void stop() {
        try {
            moveLock.unlock();
        } catch (Exception e) {
            // Ignore, some other thread has the lock
        } finally {
            elevatorMotor1.set(0);
        }
    }
}
