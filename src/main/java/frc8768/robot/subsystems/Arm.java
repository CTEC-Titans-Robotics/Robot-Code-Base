package frc8768.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
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
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Arm implements Subsystem {
    private static final SparkBaseConfig PIVOT_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final SparkBaseConfig INTAKE_CONFIG = new SparkFlexConfig()
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    private static final double angleOffset = -73.47656250000001;

    private  static final double upperBound = 125;
    private static final double lowerBound = -174;
    private ArmState currState = ArmState.ZERO;
    private final SparkFlex intakeMotor;
    private TalonFX pivotMotor;
    private final CANcoder absEncoder;
    private boolean atRotation = false;

    //OTT
    private static final double GEAR_RATIO = 6;
    private static final double CHAINTRAVEL_PER_ROT = (2.148*Math.PI)/GEAR_RATIO;

    //Chain Travel for one fill rotation os 2.148*pi (circumference) Approx. 6.744,
    // therefor 15 degrees = 0.281 rotations of gear,
    // therefor 2.248 rotations of the motor
    //private final RelativeEncoder armEncoder;



    public Arm() {
        pivotMotor = new TalonFX(19);

        intakeMotor = new SparkFlex(20, SparkLowLevel.MotorType.kBrushless);
        absEncoder = new CANcoder(22);

        //pivotMotor.configure(PIVOT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        intakeMotor.configure(INTAKE_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        LogUtil.registerDashLogger(this::dashLog);
    }

    /**
     * @return Degrees
     */
    private double getPosition() {
        return absEncoder.getPosition().getValue().in(Units.Degree) - angleOffset;
    }

    public void tick() {
        if (!MathUtil.isNear(currState.targetPosition, getPosition(), 4) && currState != ArmState.ZERO) {
            // if pos > upperbound go to else statement
            // if pos < lowerbound run if statement
            /*
            if (currState.targetPosition > getPosition() && getPosition() < upperBound) {
                pivotMotor.set(-0.14);
            } else  if (getPosition() > lowerBound) {
                pivotMotor.set(0.14);
            }
            */

            if(currState.targetPosition > getPosition() && currState == ArmState.INTAKE) {
//                pivotMotor.set(-0.3);
                pivotMotor.setVoltage(-1.05);
            } else if(currState == ArmState.INTAKE) {
//                pivotMotor.set(0.25);
                pivotMotor.setVoltage(1.2);
            } else if(currState == ArmState.L2) {
//                pivotMotor.set(0.25);
                pivotMotor.setVoltage(1.2);
            } else if(currState == ArmState.L3) {
//                pivotMotor.set(0.25);
                pivotMotor.setVoltage(1.2);
            } else if(currState == ArmState.L4) {
//                pivotMotor.set(0.25);
                pivotMotor.setVoltage(1.2);
            } else if (currState.targetPosition > getPosition()) {
//                pivotMotor.set(-0.2);
                pivotMotor.setVoltage(-1);
                atRotation = true;
            } else {
//                pivotMotor.set(0.15);
                pivotMotor.setVoltage(1);
                atRotation = true;
            }
        } else {
            stop();

        }
    }
    public boolean isAtRotation() {
        return atRotation;
    }
    public void stop() {
        if(currState == ArmState.ZERO) {
            pivotMotor.set(0);
        } else if(currState == ArmState.L2 || currState == ArmState.L3){
            pivotMotor.set(0.02);
        } else if(currState == ArmState.L1) {
            pivotMotor.set(0.04);
        } else if(currState == ArmState.L4) {
            pivotMotor.set(0.02);
        } else if(currState == ArmState.INTAKE) {
            pivotMotor.set(-0.02);
//            pivotMotor.setVoltage(1.5);
        } else {
            pivotMotor.set(-0.04);
        }
    }

    public void spinIntake(boolean outTake) {
        if (currState == ArmState.L1) {
            intakeMotor.set(outTake ? -0.3 : 0.13);
        } else {
            intakeMotor.set(outTake ? -0.2 : 0.2);
        }
    }

    public void stopIntake() {
        intakeMotor.set(0.04);
        //currState = ArmState.ZERO;
    }

    private Map<String, Object> dashLog() {
        HashMap<String, Object> map = new HashMap<>();
        map.put("Arm position", getPosition());
        map.put("Arm state", currState.name());
        return map;
    }

    public void moveToState(ArmState state) {
        currState = state;
    }

    public enum ArmState {
        ZERO(0),
        L1(-107),
        L2(-178),
        L3(-178),
        L4(-182),
        INTAKE(100),
        CORAL(-115);

        final double targetPosition;

        ArmState(double angleDegrees) {
            this.targetPosition = angleDegrees;
        }
    }
}