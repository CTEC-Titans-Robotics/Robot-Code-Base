package frc8768.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc8768.robot.util.LogUtil;

import java.util.HashMap;
import java.util.Map;

public class Climber implements Subsystem {

    private TalonFX climbMotor;

    private static final double gearRatio = 80.0;

    private static final double kMaxStatorAmps = 40.0; //Todo: Raise once safety established
    private static final double kMaxSupplyAmps = 0.0; //Todo: Raise once safety established

    private static final double kSecondsToRampVoltage = 0.1;
    private static final double Soft_Min_Angle = -10; //-40
    private static final double Hard_Min_Angle = -45; //-45
    private static final double Soft_Max_Angle = 20; //60
    private static final double Hard_Max_Angle = 50; //63

    private static final double kForwardLimitRotations = Soft_Min_Angle/360; //TODO: determine
    private static final double kReverseLimitRotations = Soft_Max_Angle/360; //TODO: determine
    private static final int kAscentGains = 0;
    private static final double kMaxVelocityRps = 5800 / 60 / gearRatio * 1.0; // TODO: check for accuracy
    private static final double kMaxAccelerationRps2 = kMaxVelocityRps / 0.25;
    private static final double kMaxJerkRps3 = kMaxAccelerationRps2 / 0.01;


    ///private TalonFXConfigurator climbMotorConfigurator = climbMotor.getConfigurator();



    public Climber() {



        climbMotor = new TalonFX(23);
    /*    CurrentLimitsConfigs currentlimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(kMaxStatorAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kMaxSupplyAmps)
                .withSupplyCurrentLimitEnable(false); //TODO: enable
        climbMotorConfigurator.apply(currentlimits);

        SoftwareLimitSwitchConfigs actuationLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(kForwardLimitRotations)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(kReverseLimitRotations)
                .withReverseSoftLimitEnable(true);
        climbMotorConfigurator.apply(actuationLimits);

*/

           // Voltage = |(10/63)*(angle-63)+12|
            //-40, 0, 63

        //climbMotor.getConfigurator().apply(new NeutralModeValue(NeutralMode.Brake));
    }
    public void lift() {
        climbMotor.setVoltage(3);
//        SmartDashboard.put("Climber VOlTS",climbMotor.getMotorVoltage());

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