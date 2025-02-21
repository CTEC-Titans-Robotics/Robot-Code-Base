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

public class GroundIndefector implements Subsystem {
    private static final double angleOffset = 39.28710937499999;
    private static final double upperBound = 5;
    private static final double lowerBound = 160;

    private static final SparkBaseConfig INTAKE_BASE_CONFIG = new SparkFlexConfig()
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final SparkBaseConfig Z_ROT_CONFIG = new SparkFlexConfig()
            .inverted(true)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);

    private final CANcoder absEncoder;
    private final SparkFlex intakeMotor, zRotMotor;

    public GroundIndefector() {
        intakeMotor = new SparkFlex(16, SparkLowLevel.MotorType.kBrushless);
        zRotMotor = new SparkFlex(15, SparkLowLevel.MotorType.kBrushless);
        absEncoder = new CANcoder(21);

        intakeMotor.configure(new SparkFlexConfig().apply(INTAKE_BASE_CONFIG), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        zRotMotor.configure(Z_ROT_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        LogUtil.registerDashLogger(this::dashLog);
    }

    private double getPosition() {
        return absEncoder.getPosition().getValue().in(Units.Degree) - angleOffset;
    }

    /**
     * Up is positive, down is negative
     */
    public void forward() {
        if(lowerBound < getPosition()) {
            stop();
        } else {
            zRotMotor.set(
                    MathUtil.clamp(-Math.abs(lowerBound/getPosition()), -0.04, 0)
            );
        }
    }

    public void backwards() {
        if(upperBound > getPosition()) {
            stop();
        } else {
            zRotMotor.set(
                    MathUtil.clamp(Math.abs(getPosition()/upperBound), 0, 0.30)
            );
        }
    }

    public void spinIntake(boolean outTake) {
        intakeMotor.set(outTake ? 0.5 : -0.30);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void stop() {
        zRotMotor.set(0.01);
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("Algae position", getPosition());
        return map;
    }
}
