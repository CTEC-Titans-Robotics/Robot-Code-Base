package frc8768.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Elevator {
    private static final SparkBaseConfig ELEVATOR_BASE_CONFIG = new SparkFlexConfig()
            .inverted(true)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkFlex elevatorMotor2, elevatorMotor1;

    public Elevator() {
        elevatorMotor1 = new SparkFlex(17, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkFlex(18, SparkLowLevel.MotorType.kBrushless);

        elevatorMotor1.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorMotor2.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    /**
     * Up is positive, down is negative
     */
    public void up() {
        elevatorMotor1.set(0.1);
        elevatorMotor2.set(-0.1);
    }

    public void down() {
        elevatorMotor1.set(-0.05);
        elevatorMotor2.set(0.05);
    }

    public void stop() {
        elevatorMotor1.set(0);
        elevatorMotor2.set(0);
    }
}
