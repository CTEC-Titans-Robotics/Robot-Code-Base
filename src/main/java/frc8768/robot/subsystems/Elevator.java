package frc8768.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Elevator implements Subsystem {

    private  final SparkBaseConfig ELEVATOR_BASE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkFlex  elevatorMotor2, elevatorMotor1;

    public  Elevator () {
        elevatorMotor2 = new SparkFlex(0 /* FIXME */,SparkLowLevel.MotorType.kBrushless);
        elevatorMotor1 = new SparkFlex(0 /* FIXME */, SparkLowLevel.MotorType.kBrushless);

        elevatorMotor2.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        elevatorMotor1.configure(new SparkFlexConfig().apply(ELEVATOR_BASE_CONFIG).follow(elevatorMotor2),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

}
