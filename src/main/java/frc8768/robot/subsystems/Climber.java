package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
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

public class Climber implements Subsystem {

    private TalonFX climbMotor;


    public Climber() {
        climbMotor = new TalonFX(23);
        //climbMotor.getConfigurator().apply(new NeutralModeValue(NeutralMode.Brake));
    }
    public void lift() {
        climbMotor.setVoltage(6.0);
    }

    public void drop() {
        climbMotor.setVoltage(-1.0);
    }

    public void hold() {
        climbMotor.setVoltage(1.0);
    }
    public void stop(){
        climbMotor.setVoltage(-0.05);
    }

    //boolean to get curr limit and if curr limit over certain number stop
}