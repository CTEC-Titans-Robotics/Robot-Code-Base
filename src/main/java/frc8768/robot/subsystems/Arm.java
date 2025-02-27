package frc8768.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
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
    private static final double angleOffset = 42.890625;

    private  static final double upperBound = 149;
    private static final double lowerBound = -165;
    private ArmState currState = ArmState.ZERO;
    private final SparkFlex intakeMotor, pivotMotor;
    private final CANcoder absEncoder;
    private final Lock intakeLock = new ReentrantLock();

    //OTT
    private static final double GEAR_RATIO = 8;
    private static final double CHAINTRAVEL_PER_ROT = (2.148*Math.PI)/GEAR_RATIO;

    //Chain Travel for one fill rotation os 2.148*pi (circumference) Approx. 6.744,
    // therefor 15 degrees = 0.281 rotations of gear,
    // therefor 2.248 rotations of the motor
    //private final RelativeEncoder armEncoder;



    public Arm() {
        pivotMotor = new SparkFlex(19, SparkLowLevel.MotorType.kBrushless);
        intakeMotor = new SparkFlex(20, SparkLowLevel.MotorType.kBrushless);
        absEncoder = new CANcoder(22);

        pivotMotor.configure(PIVOT_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
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
                pivotMotor.set(-0.17);
            } else if(currState == ArmState.INTAKE) {
                pivotMotor.set(0.17);
            } else if (currState.targetPosition > getPosition()) {
                pivotMotor.set(-0.15);
            } else {
                pivotMotor.set(0.15);
            }
        } else {
            stop();
        }
    }

    public void stop() {
        if(currState == ArmState.ZERO) {
            pivotMotor.set(0);
        } else if(currState == ArmState.L1 || currState == ArmState.L2 || currState == ArmState.L3){
            pivotMotor.set(0.02);
        } else {
            pivotMotor.set(-0.04);
        }
    }

    public void spinIntake(boolean outTake) {
        if(intakeLock.tryLock()) {
            intakeMotor.set(outTake ? -0.2 : 0.1);
        }
    }

    public void stopIntake() {
        try {
            intakeLock.unlock();
        } catch (Exception e) {
            // Ignore, some other thread has the lock
        } finally {
            intakeMotor.set(0.02);
        }
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
        L1(-50),
        L2(-50),
        L3(-50),
        L4(145),
        INTAKE(-168);

        final double targetPosition;

        ArmState(double angleDegrees) {
            this.targetPosition = angleDegrees;
        }
    }
}