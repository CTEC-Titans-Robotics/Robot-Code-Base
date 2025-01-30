package frc8768.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GroundIndefector implements Subsystem {
    private static final SparkBaseConfig INTAKE_BASE_CONFIG = new SparkFlexConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final SparkBaseConfig Z_ROT_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);

    // FIXME: Change to left or right when bot is built.
    private final SparkFlex intakeMotor1, intakeMotor2, zRotMotor;

    public GroundIndefector() {
        intakeMotor1 = new SparkFlex(0 /* FIXME */, SparkLowLevel.MotorType.kBrushless);
        intakeMotor2 = new SparkFlex(0 /* FIXME */, SparkLowLevel.MotorType.kBrushless);
        zRotMotor = new SparkFlex(0 /* FIXME */, SparkLowLevel.MotorType.kBrushless);

        intakeMotor1.configure(new SparkFlexConfig().apply(INTAKE_BASE_CONFIG), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        intakeMotor2.configure(new SparkFlexConfig().apply(INTAKE_BASE_CONFIG).follow(intakeMotor1),
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        zRotMotor.configure(Z_ROT_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
}
