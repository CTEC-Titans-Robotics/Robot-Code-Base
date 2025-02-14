package frc8768.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Leavator implements Subsystem {
    private  final SparkBaseConfig LEAVATOR_BASE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private final SparkFlex railMotor, intakeMotor;

    public  Leavator () {
        railMotor = new SparkFlex(19, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new SparkFlex(20, SparkLowLevel.MotorType.kBrushless);

        railMotor.configure(LEAVATOR_BASE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        railMotor.configure(LEAVATOR_BASE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

}
