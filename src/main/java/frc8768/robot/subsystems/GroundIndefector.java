package frc8768.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;

public class GroundIndefector implements Subsystem {
    private static final double angleOffset = 156.796875;
    private static final double angleOffset2 = 360 - angleOffset;
    private static final double upperBound = 25;
    private static final double lowerBound = 170;

    private final TalonFX zRotMotor;

    public GroundIndefector() {
        zRotMotor = new TalonFX(15);
        zRotMotor.setNeutralMode(NeutralModeValue.Brake);

        LogUtil.registerDashLogger(this::dashLog);
    }

    /**
     * Up is positive, down is negative
     */



    public void forward() {
            zRotMotor.set(-0.35);
    }

    public void backwards() {
            zRotMotor.set(0.3);
    }

    public void stop() {
        zRotMotor.set(0.0);
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        return map;
    }
}




